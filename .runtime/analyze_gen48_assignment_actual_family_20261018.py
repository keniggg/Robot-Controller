"""Pure arithmetic on one sealed trace; imports no production/ROS modules.

This diagnoses the four frozen empirical strip constraints only. It does not
qualify either top/support assignment, replace raw images or grant motion.
"""
import hashlib
import json
import math
from pathlib import Path
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
TRACE = V/'gen48_g47_exact_box_visibility_probe_r1/attempt_r5/assignment_trace_before_exact_comparison.json'
TRACE_SHA = '49b319760b13b9ccd98817b73cab3f30be64abdf3277a53858a7f77765240245'
OUT = V/'gen48_assignment_actual_family_r1'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def trig_range(a, b, lower, upper):
    """Exact sinusoid extrema on the closed interval, up to float arithmetic."""
    assert math.isfinite(a) and math.isfinite(b) and lower <= upper
    values = [a*math.cos(theta)+b*math.sin(theta) for theta in (lower, upper)]
    phase = math.atan2(b, a)
    start = math.floor((lower-phase)/math.pi)-1
    end = math.ceil((upper-phase)/math.pi)+1
    for k in range(start, end+1):
        theta = phase+k*math.pi
        if lower <= theta <= upper:
            values.append(a*math.cos(theta)+b*math.sin(theta))
    return min(values), max(values)


def projection(point, side, theta):
    c, s = math.cos(theta), math.sin(theta)
    return point[0]*c+point[1]*s if side % 2 else point[1]*c-point[0]*s


def side_axis(side):
    return 0 if side % 2 else 1


def offsets_at(groups, theta):
    result = []
    for side, group in enumerate(groups):
        values = [projection(p, side, theta) for p in group['metric_points']]
        radius = group['strip_radius_m']
        result.append([max(values)-radius, min(values)+radius])
    return result


def verify_witness(witness, groups, local, camera):
    theta = witness['theta_rad']
    assert local['common_interval'][0] <= theta <= local['common_interval'][1]
    assert -local['yaw'] <= theta <= local['yaw']
    offsets = witness['offsets_in_rotated_absolute_A_frame_m']
    intervals = offsets_at(groups, theta)
    details = []
    for side, (group, face) in enumerate(zip(groups, offsets)):
        assert intervals[side][0] < face < intervals[side][1]
        errors = [abs(projection(p, side, theta)-face) for p in group['metric_points']]
        slack = group['strip_radius_m']-max(errors)
        assert slack > 1e-8
        cp = projection(camera, side, theta)
        details.append(dict(side=side, plane_label_in_original_groups=group['plane'],
            response_count=len(errors), strip_radius_m=group['strip_radius_m'],
            maximum_absolute_point_to_face_residual_m=max(errors),
            minimum_strip_slack_m=slack, face_offset_m=face,
            feasible_face_interval_m=intervals[side], camera_projection_m=cp,
            face_minus_camera_m=face-cp))
    lo, hi = [offsets[3], offsets[0]], [offsets[1], offsets[2]]
    assert all(.005 < b-a < .6 for a, b in zip(lo, hi))
    camera_rot = [projection(camera, 1, theta), projection(camera, 0, theta)]
    states = []
    for j in (0, 1):
        states.append('outside_negative' if camera_rot[j] < lo[j] else
            'outside_positive' if camera_rot[j] > hi[j] else 'inside')
    assert states == witness['horizontal_visibility_states']
    assert local['camera_height_above_support_m'] > local['h']+local['hu']
    # These witnesses use theta=0, so containment in the original directional
    # nominal envelope has an exact simple interval form; no yaw is dropped.
    assert theta == 0.
    envelope_slack = []
    for j in (0, 1):
        low = local['lo'][j]-local['axis'][j]
        high = local['hi'][j]+local['axis'][j]
        assert low <= lo[j] < hi[j] <= high
        envelope_slack.append([lo[j]-low, high-hi[j]])
    h, d, A, T = local['h'], local['d'], local['A'], local['T']
    optical_depths = []
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in (0., h):
                base = [A[k][0]*x+A[k][1]*y+A[k][2]*(z-d) for k in range(3)]
                optical_depths.append(dot([base[k]-T[k][3] for k in range(3)],
                    [T[k][2] for k in range(3)]))
    assert min(optical_depths) > 0.
    return dict(all_four_strips_satisfied=True, exact_original_response_count=sum(
        len(g['metric_points']) for g in groups), points_by_side=details,
        lower_xy_m=lo, upper_xy_m=hi, size_xy_m=[b-a for a,b in zip(lo,hi)],
        camera_xy_in_rotated_frame_m=camera_rot,
        visibility_states=states, original_axis_envelope_slack_m=envelope_slack,
        height_m=h, height_uncertainty_m=local['hu'],
        camera_height_above_support_m=local['camera_height_above_support_m'],
        camera_above_full_height_family=True, optical_depths_m=optical_depths,
        all_eight_corners_in_front_of_camera=True)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert sha(TRACE) == TRACE_SHA
    release_path = V/'gen47_release_manifest.json'
    release = json.loads(release_path.read_text())
    assert sha(release_path) == '69e63cc71c4c63f1bbeaa2693dd88829363f06c878117f5f6129f38eaaf43e5a'
    source_before = {p: sha(W/p) for p in release['source_sha256']}
    assert source_before == release['source_sha256']
    trace = json.loads(TRACE.read_text())
    event = trace['events'][0]
    assert event['event'] == 'call' and trace['original_gate_modified'] is False
    local = dict(event['locals'])
    infer = event['infer_locals']
    groups = infer['groups']
    assert len(groups) == 4 and [g['side'] for g in groups] == list(range(4))
    assert [len(g['metric_points']) for g in groups] == [143, 89, 110, 79]
    A, T = local['A'], local['T']
    camera = [dot([T[k][3] for k in range(3)], [A[k][j] for k in range(3)]) for j in (0, 1)]
    local['common_interval'] = infer['result']['common_yaw_interval_rad']
    local['camera_height_above_support_m'] = dot([T[k][3] for k in range(3)],
        [A[k][2] for k in range(3)])+local['d']
    bounds = []
    for side, group in enumerate(groups):
        q = [[p[j]-camera[j] for j in (0,1)] for p in group['metric_points']]
        extrema = [trig_range(p[0],p[1],-local['yaw'],local['yaw']) if side % 2 else
            trig_range(p[1],-p[0],-local['yaw'],local['yaw']) for p in q]
        bounds.append(dict(side=side, full_theta_interval_rad=[-local['yaw'],local['yaw']],
            lower_camera_relative_m=max(v[0] for v in extrema)-group['strip_radius_m'],
            upper_camera_relative_m=min(v[1] for v in extrema)+group['strip_radius_m'],
            strip_radius_m=group['strip_radius_m'], response_count=len(q)))
    zero = offsets_at(groups, 0.)
    assert all(a < b for a,b in zero)
    assert zero[3][0] < camera[0] < zero[3][1]
    # Hold theta and three faces fixed. Move only lower-x inside its original
    # common strip intersection, with strict margins on each side of camera.
    common = [(a+b)/2. for a,b in zero]
    inside, outside = list(common), list(common)
    inside[3] = (zero[3][0]+camera[0])/2.
    outside[3] = (camera[0]+zero[3][1])/2.
    witnesses = []
    for label, offsets, states in (
        ('camera_inside_x_span', inside, ['inside','outside_negative']),
        ('camera_outside_negative_x', outside, ['outside_negative','outside_negative'])):
        witness = dict(label=label,theta_rad=0.,
            offsets_in_rotated_absolute_A_frame_m=offsets,
            horizontal_visibility_states=states,
            scope='Original four frozen empirical strip inequalities only; no full box/RGB/depth qualification or motion authority.')
        witness['verification'] = verify_witness(witness, groups, local, camera)
        witnesses.append(witness)
    OUT.mkdir(exist_ok=True)
    assert not (OUT/'result.json').exists()
    evidence = dict(trace=str(TRACE), trace_sha256=TRACE_SHA,
        camera_xy_in_absolute_A_frame_m=camera,
        nominal_camera_minus_lower_x_m=camera[0]-local['lo'][0],
        complete_yaw_bound_rad=local['yaw'], original_common_yaw_interval_rad=local['common_interval'],
        original_axis_uncertainty_m=local['axis'], original_height_uncertainty_m=local['hu'],
        all_original_strip_response_count=sum(len(g['metric_points']) for g in groups),
        full_symmetric_yaw_camera_relative_bounds=bounds,
        theta_zero_face_intersections_m=zero, witnesses=witnesses)
    (OUT/'arithmetic_and_witnesses.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
    assert sha(TRACE) == TRACE_SHA
    assert {p: sha(W/p) for p in source_before} == source_before
    result = dict(kind='GEN48_original_assignment_strip_family_arithmetic',passed=True,
        created_ns=time.time_ns(),review_scope='Pure arithmetic on captured original four-strip family; no production/test/ROS execution',
        helper=str(Path(__file__).resolve()),helper_sha256=sha(Path(__file__)),
        trace=str(TRACE),trace_sha256=TRACE_SHA,
        arithmetic_evidence_sha256=sha(OUT/'arithmetic_and_witnesses.json'),
        original_frozen_strip_family_really_crosses_x_visibility=True,
        crossing_witness_theta_rad=0.,
        crossing_is_only_axis_box_overapproximation=False,
        full_symmetric_yaw_strip_sign_proof_passed=False,
        y_axis_outside_negative_proved=True,
        original_top_camera_height_gate_passed=True,
        four_strip_response_count=421,witness_count=2,
        all_original_1052_sources_unchanged=True,
        complete_RGB_depth_branch_qualification_proved=False,
        physical_feasible_contact_candidate_proved=False,
        original_gate_changed=False,thresholds_or_budgets_changed=False,
        ROS_contacted=False,hardware_contacted=False,temperature_read=False,motion_commands_sent=False,
        limitations=['Witnesses satisfy frozen empirical strip constraints and original nominal directional envelope, not all full qualification gates.',
            'The original group3 remains labelled top during this arithmetic. The crossed state means that unchanged assignment cannot explain every member; opposite assignment must be reprojected and independently qualified.',
            'Raw pixel/height uncertainty is retained through the original complete strip_radius; no branch authenticity or full source-image proof is supplied by these inequalities.',
            'No matrix refit, interval shrink, yaw-floor removal, production helper invocation or new hardware sample was performed.'])
    (OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,sort_keys=True)+'\n')
    assert json.loads((OUT/'result.json').read_text()) == result
    print(json.dumps(dict(result=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),
        arithmetic_sha256=sha(OUT/'arithmetic_and_witnesses.json'),
        witness_lower_x_relative_m=[w['verification']['points_by_side'][3]['face_minus_camera_m'] for w in witnesses]),ensure_ascii=False))


if __name__ == '__main__':
    main()
