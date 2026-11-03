"""Read existing GEN9/10 audits and selected non-temperature driver evidence."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import re

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_restart_20261010T002347Z'
REC = EXT / 'grasp_records/20261010T044441.309012Z_081c1da83cbe47cc9da10a94a4578168'
AUDIT = REC / 'keyframes_audits/40934f6bbbaca8f6d676298dbedd8f397a4f8a74c81f6083bf60c67e8e2056ee.json'
OLD = EXT / 'grasp_records/20261010T042204.923624Z_7543ac3513a6431496aae778c0561e24/keyframes_audits/25f96210ea2d2d043a0a1624a492129915e81426ceaf0e25dad9e6eb9fc1428b.json'
sha = lambda data: hashlib.sha256(data).hexdigest()
load = lambda p: json.loads(p.read_text())
assert Path('/media/zhuyupei/SanStand').is_mount()
assert sha(AUDIT.read_bytes()) == AUDIT.stem
assert sha(OLD.read_bytes()) == OLD.stem
audit = load(AUDIT)
failure = audit['observation_failure_diagnostics']
search = failure['observation_orientation_search']
checked = [r for r in search['variant_results'] if r['checked']]
assert len(checked) == 6
reason = 'driver reference has no fresh actuation epoch: UNCONFIRMED:CONFIRMED_FEEDBACK_STALE'
assert all(reason in r['reason'] and 'following_support' not in r for r in checked)
fk = failure['joint_space_fk_search']
assert fk['audit']['candidate_count'] == 0 and fk['candidates'] == []
core_lines = []
for number, line in enumerate((NEW / 'core.log').open(errors='replace'), 1):
    stamp = re.search(r'\[(179160\d+\.\d+)\]', line)
    if stamp and 1791603059.4 < float(stamp.group(1)) < 1791603062.0 and any(
            token in line for token in ('Actuation state changed:', 'Rejected SDK joint feedback discontinuity')):
        core_lines.append(dict(line=number, text=line.rstrip(), sha256=sha(line.encode())))
assert any('Actuation state changed: status=UNCONFIRMED:CONFIRMED_FEEDBACK_STALE' in r['text'] for r in core_lines)
actuation_lines = []
for number, line in enumerate((NEW / 'watch.log').open(errors='replace'), 1):
    if ' ACTUATION ' in line:
        # Some synchronous log writes share one physical line. Keep just the
        # status token, preserving provenance separately for that exact line.
        status = re.search(r'ACTUATION (\w+:[A-Z_]+)', line)
        stamp = re.match(r'\d+\.\d+', line)
        if status and stamp and 1791602600. < float(stamp.group()) < 1791607506.:
            actuation_lines.append(dict(line=number, stamp_sec=float(stamp.group()),
                status=status.group(1), original_line_sha256=sha(line.encode())))
observer = next((NEW / 'gen10_handoff_analysis').glob('motion_boundary_10_*.jsonl'))
rows = [json.loads(line) for line in observer.open()]
feedback = [r for r in rows if r.get('kind') == 'accepted_q'
    and 1791607483000000000 <= r['received_ros_ns'] <= 1791607505100000000]
assert feedback
max_gap = max((b['received_ros_ns'] - a['received_ros_ns']) * 1e-9
    for a, b in zip(feedback, feedback[1:]))
report = dict(
    generation=10, audit=str(AUDIT), audit_sha256=AUDIT.stem,
    old_generation9_audit=str(OLD), old_generation9_audit_sha256=OLD.stem,
    planning_origin=audit['planning_origin'], checked_candidates=checked,
    unchecked_count=search['unchecked_variant_count'],
    unchecked_reasons=dict(Counter(r['reason'] for r in search['variant_results'] if not r['checked'])),
    endpoint_hint_summary={k: failure['unknown_observation_scheduling']['endpoint_ik_search_hint'][k]
        for k in ('checked_count', 'hinted_count', 'used_sec', 'status')},
    joint_fk_search=fk,
    generation9_checked_reasons=[r['reason'] for r in load(OLD)['observation_failure_diagnostics'][
        'observation_orientation_search']['variant_results'] if r['checked']],
    first_stale_transition=dict(stamp_sec=1791603061.388772285,
        status='UNCONFIRMED:CONFIRMED_FEEDBACK_STALE',
        source=str(NEW / 'core.log'), line=40574,
        preceding_rejection='Repeated Joint3 discontinuity -12.4 deg to -25.0 deg, delta 0.220893 rad exceeds approximately 0.0423 rad accepted-feedback allowance.'),
    exact_non_temperature_driver_excerpt=core_lines,
    actuation_observer_status_lines=actuation_lines,
    generation10_feedback_interval=dict(observer=str(observer), selected_samples=len(feedback),
        first_received_ns=feedback[0]['received_ros_ns'], last_received_ns=feedback[-1]['received_ros_ns'],
        maximum_received_gap_sec=max_gap,
        maximum_motion_rad=max(r['maximum_delta_rad'] for r in feedback)),
    findings=[
        'GEN10 six checked endpoints fail at the same gateway actuation-reference gate. Outer MOVEIT_UNREACHABLE is generic and does not establish IK unreachability. No following_support evaluation was entered.',
        'FK generation ran, but its two-second shared budget expired in the first seed of the first direction. It produced zero accepted candidates, so no successful FK joint proof or seed was dropped in this record.',
        'The first stale transition predates GEN9 and GEN10. Driver logs show accepted-feedback rejection immediately before it; later fresh stationary feedback does not itself restore a CONFIRMED state because the driver keeps the invalidated actuation state latched.',
        'GEN9 four strict failures are OBSERVATION_POSE_PLAN_FAILED, not the GEN10 actuation message. The motion handler runs before preview evidence; those earlier failures can hide a later common reference failure. Do not rewrite GEN9 as a demonstrated actuation-gate rejection.',
        'GEN10 accepted joint feedback is fresh and stationary during the observed attempt. Recovery must use the existing real positive-enable/epoch/confirmation route; no timestamp, confirmed status or freshness threshold should be fabricated.',
    ],
    limitations=[
        'The recorder was still active when this analysis began; no unindexed bag was modified or used as sealed evidence.',
        'Driver transition excerpts establish software state and rejected raw feedback, not whether the anomalous raw joint value represented physical motion or a transport/firmware fault.',
        'This read-only analysis does not execute a recovery, establish a collision-free path, or prove physical grasp success.',
    ],
    physical_actions=False, production_edited=False, temperature_queried=False)
output = NEW / 'gen10_handoff_analysis/strict_actuation_failure_assessment.json'
assert not output.exists()
output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
print(json.dumps(dict(path=str(output), sha256=sha(output.read_bytes()),
    checked=len(checked), fk_accepted=0, feedback_samples=len(feedback),
    maximum_feedback_gap_sec=max_gap, motion_rad=report['generation10_feedback_interval']['maximum_motion_rad']), ensure_ascii=False))
