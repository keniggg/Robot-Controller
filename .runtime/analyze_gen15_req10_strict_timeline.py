"""Closed GEN15 record and text logs only. Never contacts a ROS master."""
from collections import Counter
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import re

import rosbag
import rospy

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = ROOT / 'ros_log/full_latest_no_skin_20261011T000625Z'
RECORD = ROOT / 'grasp_records/20261011T110808.411096Z_6fb1906474c24e59b6be584da92c6f43'
AUDIT = RECORD / 'keyframes_audits/abd232c08c8a42cebe5b6088516b6bc4c9905077a1f786941adbf48e52a6b11a.json'
WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
sha = lambda b: hashlib.sha256(b).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount()
assert sha(AUDIT.read_bytes()) == AUDIT.stem
record = json.loads(AUDIT.read_text())
d = record['observation_failure_diagnostics']
search = d['observation_orientation_search']
checked = [row for row in search['variant_results'] if row.get('checked')]
assert len(checked) == 4
assert all(row['search_family'] == 'camera_orientation_lattice' for row in checked)
assert all(row['search_family'] == 'camera_orientation_lattice' for row in d['observation_candidates'])

logs = {}
ansi = re.compile(r'\x1b\[[0-9;]*m')
for name in ('core.log', 'gen14_observation_search_compute_deployment/consumers.log'):
    selected = []
    for i, raw in enumerate((NEW / name).read_text(errors='replace').splitlines(), 1):
        line = ansi.sub('', raw)
        stamps = re.findall(r'\[(17917169\d+\.\d+)\]', line)
        if not stamps:
            continue
        ns = int(Decimal(stamps[0]) * 10**9)
        if not 1791716945000000000 <= ns < 1791716963000000000:
            continue
        if name == 'core.log':
            keep = any(s in line for s in ('check_pose_strict', 'goal tree', 'Timed out',
                'ABORTED: TIMED_OUT', 'Created 1 states', 'No solution found', 'Unable to solve'))
        else:
            keep = any(s in line for s in ('UNKNOWN_OBSERVATION_NO_SAFE_VIEW', 'ERROR', 'observation first FK',
                'OBSERVATION_SEARCH_INCOMPLETE', 'Unknown target first loss', 'station_invalidated'))
            if 'pipeline metrics:' in line:
                keep = False
        if keep:
            selected.append(dict(line=i, stamp_ns=ns, text=line))
    logs[name] = dict(path=str(NEW / name), selected_lines=selected,
        selected_lines_sha256=sha(json.dumps(selected, sort_keys=True).encode()))

requests = sorted([r for r in logs['core.log']['selected_lines'] if 'check_pose_strict request' in r['text']], key=lambda r:r['stamp_ns'])
results = sorted([r for r in logs['core.log']['selected_lines'] if 'check_pose_strict result' in r['text']], key=lambda r:r['stamp_ns'])
assert len(requests) == len(results) == 4
calls = []
for index, (request, result, variant) in enumerate(zip(requests, results, checked)):
    assert request['stamp_ns'] < result['stamp_ns']
    assert 'OBSERVATION_POSE_PLAN_FAILED' in result['text']
    errors = [r for r in logs['core.log']['selected_lines']
        if request['stamp_ns'] <= r['stamp_ns'] <= result['stamp_ns'] and 'goal tree' in r['text']]
    assert errors
    candidate_matches = [r for r in d['observation_candidates']
        if all(r[k] == variant[k] for k in ('camera_target_distance_m', 'camera_roll_offset_deg', 'camera_tilt_offset_deg'))]
    # The core API logs tool XYZ to 3 decimals; retain that precision explicitly.
    xyz = re.search(r'xyz=\(([^)]+)\)', request['text']).group(1)
    rounded = [float(v.strip()) for v in xyz.split(',')]
    candidate_matches = [r for r in candidate_matches if all(abs(r['base_from_tool'][i][3] - rounded[i]) <= .00050000001 for i in range(3))]
    assert len(candidate_matches) == 1
    calls.append(dict(strict_index=index, candidate_index=candidate_matches[0]['candidate_index'],
        request=request, response=result, duration_sec=(result['stamp_ns']-request['stamp_ns'])/1e9,
        low_level_goal_failure=errors, frozen_variant_result=variant,
        full_tool_target=candidate_matches[0]['base_from_tool'],
        following_path_check_reached=False, endpoint_joint_hint_used=False))

changes = []
last = None
with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
    for topic, message, receive in bag.read_messages(topics=['/joint_states'],
            start_time=rospy.Time(1791716948), end_time=rospy.Time(1791716954)):
        q = tuple(message.position)
        if q != last:
            changes.append(dict(topic=topic, receive_stamp_ns=receive.to_nsec(),
                source_stamp_ns=message.header.stamp.to_nsec(), joint_names=list(message.name), positions=list(q)))
            last = q
fk = d['joint_space_fk_search']
seed = fk['audit']['current_joint_positions']
post = d['moveit_input_joint_state']['position_rad'][:6]
delta = [b-a for a,b in zip(seed, post)]
assert len(changes) == 2 and changes[0]['positions'][:6] == seed and changes[1]['positions'][:6] == post
assert delta[0] != 0 and all(v == 0 for v in delta[1:])
assert fk['audit']['candidate_count'] == 1 and fk['candidates'] == [] and not fk['cached_inputs_validated']
source = WORK / 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
lines = source.read_text().splitlines()
comparison_lines = [(i+1,line) for i,line in enumerate(lines) if "measured_seed()[0] != q" in line]
assert len(comparison_lines) == 1
i = comparison_lines[0][0]
excerpt = [dict(line=j+1,text=lines[j]) for j in range(i-6,i+6)]
report = dict(source_audit=str(AUDIT), source_audit_sha256=AUDIT.stem,
    request_id=record['request_id'], source_stamp_ns=record['snapshot_stamp_ns'],
    actual_strict_calls=calls, strict_elapsed_sum_sec=sum(r['duration_sec'] for r in calls),
    checked=4, configured=222, unchecked=218,
    actual_candidate_families=dict(Counter(v['search_family'] for v in d['observation_candidates'])),
    endpoint_rejection_counts=dict(Counter(v['failure_code'] for v in d['observation_endpoint_rejections'])),
    fk_generation=dict(available=fk['available'], internal_accepted_candidate_count=fk['audit']['candidate_count'],
        usable_cached_candidate_count=len(fk['candidates']), cached_inputs_validated=fk['cached_inputs_validated'],
        status=fk['audit']['status'], used_sec=fk['used_sec'], seed_stamp_ns=fk['audit']['joint_seed_stamp_ns'],
        seed_joints_rad=seed, actual_seed_generation_directions=fk['audit']['directions']),
    measured_joint_changes=changes, post_generation_joint_state=d['moveit_input_joint_state'],
    measured_delta_rad=delta,
    invalidation_source=dict(path=str(source), sha256=sha(source.read_bytes()), excerpt=excerpt),
    logs=logs,
    conclusion=[
        'All four actual strict requests were camera_orientation_lattice targets and failed while MoveIt attempted to sample goal states; no resulting trajectory reached continuous following/support qualification.',
        'The 222 count is the actual all-lattice set, not 220 lattice plus two FK proposals.',
        'The FK search internally accepted one endpoint but its usable cache has zero proposals and key validation is false. No FK branch was submitted to strict.',
        'One real encoder count on Joint1 changed 0.281 s after the frozen FK seed. The existing exact-q post-solver comparison rejects that changed seed.',
        'The cache stores the pre-postcheck report. Therefore its budget_exhausted status obscures the subsequently returned inputs_changed code; exact failure-code handoff is not retained in this audit.'
    ],
    limitations=[
        'Joint-state mismatch is directly recorded and sufficient to trigger the exact-q branch; no assertion is made that other dynamic parameters could not also have changed.',
        'Goal-state sampling failure alone does not prove collision, mathematical impossibility or absence of other reachable observations.',
        'Read-only closed bag and log analysis. No ROS services, publications, motion or production source edits.'
    ])
out = NEW / 'gen15_analysis/req10_strict_and_fk_cache_timeline.json'
out.parent.mkdir(exist_ok=True)
assert not out.exists()
out.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(dict(path=str(out), sha256=sha(out.read_bytes()), strict_calls=len(calls), fk_entered_strict=False,
    joint_delta_rad=delta, total_strict_sec=report['strict_elapsed_sum_sec'])))
