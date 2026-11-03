"""GEN16 JSON/text evidence only; deliberately never opens the recording bag."""
from collections import Counter
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import re

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_no_skin_20261011T000625Z'
REC = EXT / 'grasp_records/20261011T113712.131215Z_6bfa27d92a2944fbb1b445f2c8ebdd6f'
sha = lambda b: hashlib.sha256(b).hexdigest()
first_path = REC / 'keyframes_audits/3dc5acce3ac9193881ae0fe1defe64667e43f15150bb065c89832ceed804762b.json'
last_path = REC / 'keyframes_audits/87c4f588d028eb67065a14ea82b7f71ee63ffe5056db5aa7d32798dd0009b10a.json'
assert sha(first_path.read_bytes()) == first_path.stem
assert sha(last_path.read_bytes()) == last_path.stem
first, last = json.loads(first_path.read_text()), json.loads(last_path.read_text())
selected = first['selected']; view = selected['observation_view']
search = view['proposal_search']; proposal = view['proposal']
assert first['request_id'] == 2 and last['request_id'] == 13
assert view['search_family'] == 'joint_space_fk'
assert search['generated_candidate_count'] == search['candidate_count'] == 4
assert search['seed_handoff']['delta_rad'] == [0.] * 6
selected_direction = [d for d in search['directions'] if d['gap_index'] == proposal['direction_gap_index']
    and d['exposure_target_noise_multiplier'] == proposal['exposure_target_noise_multiplier']]
assert len(selected_direction) == 1 and selected_direction[0]['solver_success'] is True
logs = {}; contact_failures = []; final_candidates = []; tasks = []; leases = []
for name in ('core.log', 'gen15_seed_handoff_compute_deployment/consumers.log'):
    entries = []
    for number, raw in enumerate((NEW / name).read_text(errors='replace').splitlines(), 1):
        line = re.sub(r'\x1b\[[0-9;]*m', '', raw)
        m = re.search(r'\[(1791718\d+\.\d+)\]', line)
        if not m:
            continue
        ns = int(Decimal(m[1]) * 10**9)
        if not 1791718633000000000 <= ns <= 1791718775000000000:
            continue
        receipt = dict(line=number, stamp_ns=ns, line_sha256=sha(raw.encode()))
        if name == 'core.log':
            if 'check_pose_strict request' in line or 'check_pose_strict result' in line:
                entries.append(dict(receipt, text=line))
            continue
        if 'rejected contact evidence:' in line:
            d = json.loads(line.split('rejected contact evidence: ', 1)[1]); precheck = d['precheck']
            contact_failures.append(dict(receipt, plan_id=d['plan_id'], request=d['request'],
                source_stamp_ns=d['source_stamp_ns'], required_open_width_m=d['required_open_width_m'],
                checked_stage_names=precheck.get('checked_stage_names'), failed_stage=precheck.get('failed_stage'),
                failure_code=precheck.get('failure_code'), reason=precheck.get('reason'), success=precheck.get('success')))
        elif 'remote 6D final candidate rejected:' in line:
            final_candidates.append(dict(receipt, text=line))
        elif '[Grasp]' in line:
            tasks.append(dict(receipt, text=line))
        elif 'Observe-return measurement request:' in line:
            d = json.loads(line.split('Observe-return measurement request: ', 1)[1])
            leases.append(dict(receipt, state=d['state'], view_index=d['view_index'],
                motion_kind=d.get('motion_kind'), request_id=d['request_id'],
                view_plan_id=d.get('view_plan_id'), source_stamps_ns=d.get('source_stamps_ns')))
    logs[name] = dict(path=str(NEW / name), strict_entries=entries)

assert last['outcome']['code'] == 'NEAR_FIELD_DIRECT_TIMEOUT'
assert len([x for x in leases if x['state'] == 'executing']) == 1
assert len([x for x in leases if x['state'] == 'qualified']) == 1
assert contact_failures and all(x['failed_stage'] == 'grasp' for x in contact_failures)
report = dict(record=str(REC), bag_opened=False,
    first_observation=dict(audit_path=str(first_path), audit_sha256=first_path.stem,
        request_id=2, source_stamp_ns=first['snapshot_stamp_ns'], plan_id=first['plan_id'],
        search_family=view['search_family'], generation=search,
        selected_direction=selected_direction[0], selected_proposal=proposal,
        chosen_distance_m=view['camera_target_distance_m'], chosen_tilt_deg=view['camera_tilt_offset_deg'],
        chosen_roll_deg=view['camera_roll_offset_deg'],
        strict_moveit_summary={k:v for k,v in selected['moveit'].items() if k != 'reason'}),
    final_request=dict(audit_path=str(last_path), audit_sha256=last_path.stem,
        request_id=13, source_stamp_ns=last['snapshot_stamp_ns'], outcome=last['outcome'],
        stage_counts=last['pipeline_funnel']['stage_counts']),
    strict_logs=logs, measurement_requests=leases, task_timeline=tasks,
    contact_rejection_receipts=contact_failures, final_candidate_rejections=final_candidates,
    contact_failure_codes=dict(Counter(x['failure_code'] for x in contact_failures)),
    contact_failure_reasons=dict(Counter(x['reason'] for x in contact_failures)),
    conclusions=[
        'One FK-derived observation was planned, executed and qualified. No second observation execution request was recorded before task failure.',
        'The selected candidate was gap 1 / exposure tier 2, whose numerical solve converged normally. Separate same-search candidates used early qualification.',
        'All six generation-to-handoff joint deltas were zero. This run does not demonstrate a nonzero one-count tolerance event.',
        'After the first observation, contact candidates and height retries reached strict pregrasp/approach checks but failed grasp-stage predicted_command continuous gripper/target geometry.',
        'The final task result is NEAR_FIELD_DIRECT_TIMEOUT after the fixed contact phase deadline, not a second-station FK proposal handoff rejection.'
    ], limitations=[
        'Full-arm reachability or infeasibility of untested contact candidates cannot be inferred from this bounded attempt.',
        'This report references immutable audits and preserves selected growing-log line hashes, not a purported final hash for still-open ROS log files.',
        'No active or closed bag was opened, and no ROS service/publication/robot action/source edit was performed.'
    ])
out = NEW / 'gen16_analysis/fk_provenance_and_contact_terminal.json'
assert Path('/media/zhuyupei/SanStand').is_mount() and not out.exists()
out.parent.mkdir(exist_ok=True)
out.write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(dict(path=str(out), sha256=sha(out.read_bytes()),
    contact_rejection_receipts=len(contact_failures), final_candidate_rejections=len(final_candidates),
    observation_executing_leases=1, observation_qualified_leases=1,
    selected_fk_gap=proposal['direction_gap_index'], selected_fk_tier=proposal['exposure_target_noise_multiplier'])))
