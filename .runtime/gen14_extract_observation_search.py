"""Offline extraction only; never imports rospy or contacts a ROS service."""
import collections
import hashlib
import json
import pathlib
import re

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = ROOT / 'ros_log/full_latest_no_skin_20261011T000625Z'
REC = ROOT / 'grasp_records/20261011T070957.732968Z_f55879ef6b7140c3a5b3ebe7ec00e0d7'
OUT = NEW / 'gen14_analysis'
AUD = REC / 'keyframes_audits/2b08923465a42090a121778fb20d63c8537eb596b365f00d0ca77abd001e3719.json'
WORK = pathlib.Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert json.loads((REC / 'recorder_outcome.json').read_text())['bag_closed'] is True
    d = json.loads(AUD.read_text())
    a = d['observation_failure_diagnostics']
    s = a['unknown_observation_scheduling']
    o = a['observation_orientation_search']
    dispatch = s['first_incumbent_coverage']['dispatched_original_indices']
    ordered = {r['observation_view']['check_scheduling_index']: r
               for r in a['observation_candidates']}
    rows = []
    for index in dispatch:
        r = ordered[index]
        v = r['observation_view']
        m = v['check_scheduling_metrics']
        direction = v.get('observe_return_directed_missing_side', {})
        transform = np.asarray(r['base_from_tool'])
        result = o['variant_results'][len(rows)]
        for key in ('camera_target_distance_m', 'camera_tilt_offset_deg', 'camera_roll_offset_deg'):
            assert abs(r[key] - result[key]) < 1e-10
        rows.append(dict(
            candidate_index=r['candidate_index'], scheduling_index=index,
            target_base_from_tool=transform.tolist(),
            target_pose_xyz_xyzw=list(transform[:3, 3]) + list(Rotation.from_dcm(transform[:3, :3]).as_quat()),
            frame_id=r['frame_id'], source_stamp_ns=d['snapshot_stamp_ns'],
            distance_m=r['camera_target_distance_m'], tilt_deg=r['camera_tilt_offset_deg'],
            roll_deg=r['camera_roll_offset_deg'], missing_side_angle_deg=direction.get('angle_deg'),
            missing_side_sign=direction.get('missing_side_sign'),
            rotation_axis_base=direction.get('rotation_axis_base'),
            support_status=m.get('measured_support_schedule', {}).get('status'),
            predicted_information_utility=m.get('predicted_information_utility'),
            endpoint_hint=m.get('endpoint_ik_hint'), original_strict_result=result))
    assert len(rows) == 220 and sum(r['original_strict_result']['checked'] for r in rows) == 4
    selected = rows[:4]
    for roll in (15., 30., -30., 45., -45., 90., -90., 180.):
        group = [r for r in rows[4:] if r['roll_deg'] == roll
                 and r['support_status'] == 'sufficient' and r['predicted_information_utility'] > 0.]
        if group:
            selected.append(min(group, key=lambda r: (abs(r['distance_m'] - .285), r['tilt_deg'], r['scheduling_index'])))
    logs = []
    for number, line in enumerate((NEW / 'core.log').open(errors='replace'), 1):
        match = re.search(r'\[(17917026\d+\.\d+)\]', line)
        if (match and 1791702659 <= float(match.group(1)) <= 1791702663
                and any(token in line for token in ('check_pose_strict', 'goal tree', 'Timed out', 'ABORTED'))):
            logs.append(dict(line_number=number, stamp_sec=float(match.group(1)), text=line.strip()))
    starts = [r['stamp_sec'] for r in logs if 'request execute=False' in r['text']]
    ends = [r['stamp_sec'] for r in logs if 'result success=False' in r['text']]
    durations = [end - start for start, end in zip(starts, ends)]
    assert len(durations) == 4
    model = WORK / 'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
    inputs = dict(
        schema=1, source_audit=str(AUD), source_audit_sha256=sha(AUD), record_closed=True,
        model_path=str(model), model_sha256=sha(model),
        model_matches_recorded_hint=sha(model) == s['endpoint_ik_search_hint']['robot_description_sha256'],
        group_name='alicia', ik_link_name='tool0', frame_id='base_link',
        recorded_joint_seed=s['endpoint_ik_search_hint']['joint_seed'],
        recorded_strict_joint_seed=a['moveit_input_joint_state'],
        original_ik_timeout_sec=.02, original_ik_avoid_collisions=True,
        read_only_selected_probe_count=len(selected), selected_probes=selected,
        all_220_in_original_first_path_dispatch_order=rows,
        limitations=[
            'Inputs do not authorize action, scene change, trajectory, publishers, or MoveGroup execution.',
            'Later live compute_ik/check_state_validity queries use the current collision scene, not the historical scene.',
            'Original collision-aware 20ms NO_IK_SOLUTION does not distinguish kinematic failure from collision rejection.',
            'Recorded sample timestamps are provenance; a new diagnostic service call must not pretend they are fresh execution evidence.'])
    summary = dict(
        source_audit=str(AUD), source_audit_sha256=sha(AUD), actual_checked=rows[:4],
        strict_request_result_duration_sec=durations, strict_duration_sum_sec=sum(durations),
        strict_budget=dict(start_new_check_seconds=3., maximum_count=6,
                           first_three_duration_sec=sum(durations[:3]),
                           fourth_completes_after_budget=True,
                           post_incumbent_comparison_seconds=8., incumbent_found=False),
        unchecked_count=216, unchecked_reasons=dict(collections.Counter(
            r.get('reason') for r in o['variant_results'] if not r['checked'])),
        diverse_schedule_actual=True, coverage_first_four_indices=dispatch[:4],
        fk_search=a['joint_space_fk_search'], endpoint_hint_summary=s['endpoint_ik_search_hint'],
        measured_support_summary=s['measured_support_schedule'], search_complete=o['search_complete'],
        limitations=['All 220 dispatch entries include skipped budget tail; checked=true is the actual strict-call count.',
                     'No valid goal-tree sample in four bounded planner calls does not prove global unreachability.'])
    OUT.mkdir(exist_ok=True)
    for name, content in (
            ('req16_read_only_ik_probe_inputs.json', inputs),
            ('req16_search_failure_analysis.json', summary),
            ('req16_strict_core_excerpt.json', logs)):
        path = OUT / name
        path.write_text(json.dumps(content, indent=2) + '\n')
        print(str(path), sha(path))
    print('SELECTED', [(r['candidate_index'], r['distance_m'], r['tilt_deg'], r['roll_deg']) for r in selected])
    print('DURATIONS', durations, sum(durations))


if __name__ == '__main__':
    main()
