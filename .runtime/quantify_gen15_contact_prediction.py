#!/usr/bin/env python3
"""Pure sealed-data endpoint reconstruction; no ROS calls or motion authority."""
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_matrix

from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _stage_boxes, _box_corners, _obb_overlap)

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION = EXT/'ros_log/full_latest_ros_20261009T003407Z'
AUDIT = EXT/'grasp_records/20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70/keyframes_audits/ccf9c299b44045d82cdca1488d0329d269b6f8d414640192daf1bffcb088b410.json'
MODEL = EXT/'ros_log/user_grasp_20261008T090751Z/source_snapshot_review/gen28_fk_ik_100ms.model.urdf'
OUT = SESSION/'gen15_contact_prediction'


def sat_gap(ca, ra, sa, cb, rb, sb):
    axes = [*ra.T, *rb.T] + [np.cross(a, b) for a in ra.T for b in rb.T]
    gaps = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-10:
            continue
        axis = axis/norm
        gaps.append(abs(float((ca-cb)@axis))
                    - float(abs(ra.T@axis)@(sa/2))
                    - float(abs(rb.T@axis)@(sb/2)))
    return max(gaps)


def main():
    data = json.loads(AUDIT.read_text())
    geom = data['replay_geometry']
    center = np.array(geom['obb_center_base_m'])
    rotation = np.array(geom['R_base_obb'])
    size = np.array(geom['obb_size_xyz_m'])
    normal = np.array(geom['support_normal_base'])
    offset = geom['support_offset_m']
    points = np.array(geom['object_points_base_m'])
    registration = np.array(data['multiview_surface']['latest_registration']['snapshot_to_reference_transform'])
    rc = registration[:3,:3]@center+registration[:3,3]
    rr = registration[:3,:3]@rotation
    rp = points@registration[:3,:3].T+registration[:3,3]
    gripper = GripperGeometry(.05, .002, np.array([.0434,.0286,.06]), np.array([.1175,.155,.0774]), .003)
    fk = SerialUrdfFk(MODEL.read_text(), tuple('Joint%d'%i for i in range(1,7)))

    def report(T, opening, c=rc, r=rr, ps=rp):
        bodies = []
        for name, bc, bs in _stage_boxes(T, gripper, opening, 'y', 'z'):
            inside = np.all(np.abs((ps-bc)@T[:3,:3]) <= bs/2+1e-12, axis=1)
            bodies.append(dict(body=name, overlap=bool(_obb_overlap(bc,T[:3,:3],bs,c,r,size)),
                sat_best_separating_gap_m=sat_gap(bc,T[:3,:3],bs,c,r,size),
                measured_points_inside=int(inside.sum()),
                support_clearance_m=float(np.min(_box_corners(bc,T[:3,:3],bs)@normal+offset))))
        return dict(full_obb_jaw_width_m=float(abs(r.T@T[:3,1])@size),
            target_center_tool_y_m=float((c-T[:3,3])@T[:3,1]), opening_m=opening, bodies=bodies)

    rows = []
    for index, entry in enumerate(data['stable_evaluations']):
        check = entry.get('contact_sequence_precheck')
        if not check:
            continue
        sample = check['source_sample']
        assert sample['model_sha256'] == fk.model_sha256
        stage = entry['execution_sequence']['stages'][2]
        current = quaternion_matrix(stage['quaternion_xyzw'])
        current[:3,3] = stage['position_m']
        goal = registration@current
        raw = report(current,.05,center,rotation,points)
        before_lift = report(goal,.05)
        recorded_clearances = dict(entry['final_contact_height_margin']['nominal']['support_clearances_m'])
        shifts = [recorded_clearances[b['body']]-b['support_clearance_m'] for b in before_lift['bodies']]
        assert max(shifts)-min(shifts)<1e-9, shifts
        support_lift = sum(shifts)/len(shifts)
        assert support_lift >= -1e-9
        goal[:3,3] += normal*max(0.,support_lift)
        assert max(abs(recorded_clearances[b['body']]-b['support_clearance_m'])
                   for b in report(goal,.05)['bodies'])<1e-9
        probe = entry['endpoint_ik_search_probe']['stages'][0]['joint_solution']
        seed = np.array(probe['position_rad'][:6])
        def residual(q):
            pose = fk(q)
            return np.r_[pose[:3,3]-goal[:3,3], .2*Rotation.from_dcm(goal[:3,:3].T@pose[:3,:3]).as_rotvec()]
        fit = least_squares(residual,seed,bounds=(fk.limits[:,0],fk.limits[:,1]),
            max_nfev=100,ftol=1e-12,xtol=1e-12,gtol=1e-12)
        q = fit.x
        nominal = fk(q)
        delta = np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
        command = fk(q+delta)
        position_error = float(np.linalg.norm(nominal[:3,3]-goal[:3,3]))
        angle_error = float(np.linalg.norm(Rotation.from_dcm(goal[:3,:3].T@nominal[:3,:3]).as_rotvec()))
        assert position_error<1e-7 and angle_error<1e-6,(index,position_error,angle_error)
        rows.append(dict(index=index,source_index=entry['source_index'],variant_index=entry['variant_index'],
            recorded_reason=check['reason'],recorded_necessary_rejected=entry['contact_goal_necessary_check']['rejected'],
            same_frame_before_registration=raw,registered_nominal_requested=report(goal,check['opening_m']),
            support_lift_inferred_from_all_three_recorded_final_clearances_m=support_lift,
            offline_ik=dict(seed=seed.tolist(),q=q.tolist(),nfev=fit.nfev,position_residual_m=position_error,
                orientation_residual_rad=angle_error,actual_strict_trajectory_reproduced=False),
            bias_sdk_minus_encoder_rad=delta.tolist(),command_q_within_limits=bool(np.all(q+delta>=fk.limits[:,0]) and np.all(q+delta<=fk.limits[:,1])),
            nominal_ik_endpoint=report(nominal,check['opening_m']),
            predicted_command_endpoint=report(command,check['opening_m']),
            nominal_T=nominal.tolist(),predicted_command_T=command.tolist(),
            tool_position_shift_m=(command[:3,3]-nominal[:3,3]).tolist(),
            tool_position_shift_norm_m=float(np.linalg.norm(command[:3,3]-nominal[:3,3])),
            tool_shift_along_nominal_jaw_m=float((command[:3,3]-nominal[:3,3])@nominal[:3,1]),
            tool_orientation_shift_rad=float(np.linalg.norm(Rotation.from_dcm(nominal[:3,:3].T@command[:3,:3]).as_rotvec()))))
    def extent(xs): return [min(xs),max(xs)]
    summary = dict(candidate_count=len(rows),reasons=dict(Counter(r['recorded_reason'] for r in rows)),
        raw_obb_jaw_width_range_m=extent([r['same_frame_before_registration']['full_obb_jaw_width_m'] for r in rows]),
        registered_obb_jaw_width_range_m=extent([r['registered_nominal_requested']['full_obb_jaw_width_m'] for r in rows]),
        opening_m=sorted(set(r['registered_nominal_requested']['opening_m'] for r in rows)),
        nominal_endpoint_candidates_overlapping=sum(any(b['overlap'] for b in r['nominal_ik_endpoint']['bodies']) for r in rows),
        predicted_command_endpoint_candidates_overlapping=sum(any(b['overlap'] for b in r['predicted_command_endpoint']['bodies']) for r in rows),
        tool_shift_norm_range_m=extent([r['tool_position_shift_norm_m'] for r in rows]),
        jaw_shift_range_m=extent([r['tool_shift_along_nominal_jaw_m'] for r in rows]),
        support_lift_range_m=extent([r['support_lift_inferred_from_all_three_recorded_final_clearances_m'] for r in rows]),
        maximum_ik_position_residual_m=max(r['offline_ik']['position_residual_m'] for r in rows))
    result = dict(scope='sealed exact geometry and measured bias with independently solved endpoints; not failed trajectory reproduction or motion authority',
        audit=str(AUDIT),audit_sha256=hashlib.sha256(AUDIT.read_bytes()).hexdigest(),model_sha256=fk.model_sha256,
        source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),)},
        registration=registration.tolist(),geometry={k:v for k,v in geom.items() if k!='object_points_base_m'},
        summary=summary,rows=rows,limitations=[
          'Failed service audit retains the exact following sample and rejection label, but no complete nominal or compensated trajectory or endpoint q. Offline IK is a branch-seeded counterfactual, not exact strict-path replay.',
          'Analytical box collisions are conservative guard results, not mesh or physical contact measurements.',
          'The production deliberately checks both nominal and SDK-command FK paths; constant-bias prediction is not a hardware-response guarantee.',
          'No ROS services or hardware calls made.'])
    OUT.mkdir(parents=True,exist_ok=True)
    destination=OUT/'endpoint_quantification.json'
    destination.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(result=str(destination),sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),summary=summary),indent=2))


if __name__=='__main__': main()
