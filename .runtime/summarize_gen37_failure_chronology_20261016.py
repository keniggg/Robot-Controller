"""Read only the closed chain index; distinguish source and receipt clocks."""
import hashlib
import json
from pathlib import Path

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D = E / 'ros_log/full_latest_gen36_restart_20261017T012037Z'
source = D / 'verification/gen37_actual_attempt1_chain_index_r2/result.json'
output = D / 'verification/gen37_actual_attempt1_chronology_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
raw = source.read_bytes()
digest = hashlib.sha256(raw).hexdigest()
assert digest == 'ec142e45111c7f8cfe778ccdaabd357a7b977871f72c88509a59232ff3d8a27a'
data = json.loads(raw)
terminal = data['first_task_terminal']
terminal_ns = terminal['event_ns']
events = data['events']

def evidence(event):
    received = event['locator'].get('recorded_ns')
    source_ns = event['binding'].get('source_ns')
    return dict(event_id=event['event_id'], topic=event['topic'],
                source_ns=source_ns, event_ns=event['event_ns'],
                recorded_ns=received,
                source_relative_task_terminal_ms=(source_ns-terminal_ns)/1e6
                    if source_ns is not None else None,
                recorded_relative_task_terminal_ms=(received-terminal_ns)/1e6
                    if received is not None else None,
                summary=event['summary'], locator=event['locator'])

contact = [e for e in events if e['stage'] == 'contact_plan']
assert len(contact) == 1
lost = sorted([e for e in events
    if e['topic'] == '/grasp_6d/plan_enriched'
    and e['summary'].get('code') == 'TARGET_LOST'
    and e['locator'].get('recorded_ns', 0) > 1792207340733134031],
    key=lambda e: e['locator']['recorded_ns'])
detector = sorted([e for e in events
    if e['topic'] == '/perception/unknown/detector_status'
    and e['summary'].get('target_lost')],
    key=lambda e: e['locator']['recorded_ns'])
assert lost and detector
assert contact[0]['locator']['recorded_ns'] < terminal_ns
assert all(e['locator']['recorded_ns'] > terminal_ns for e in lost + detector)
output.mkdir(exist_ok=False)
result = dict(passed=True, diagnostic_only=True, motion_authorized=False,
    source_index=str(source), source_sha256=digest,
    contact_plan_rejection=evidence(contact[0]), task_terminal=evidence(terminal),
    first_detector_target_lost=evidence(detector[0]),
    plan_target_lost_near_terminal=[evidence(e) for e in lost
        if e['locator']['recorded_ns']-terminal_ns < 500_000_000],
    conclusion='Recorded contact-plan rejection precedes task failure. All '
        'indexed later TARGET_LOST deliveries follow task terminal. A source '
        'stamp about 365 ms earlier is not the publication or delivery time.',
    limitations=['Bag receipt order is not a precise callback execution trace; '
        'no inference about unrecorded internal events is made.'],
    ros_initialized=False, temperature_values_read=False,
    motion_commands_sent=False)
assert source.read_bytes() == raw
(output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(passed=True, output=str(output),
    first_lost_receipt_after_terminal_ms=(lost[0]['locator']['recorded_ns']-terminal_ns)/1e6,
    first_detector_lost_receipt_after_terminal_ms=(detector[0]['locator']['recorded_ns']-terminal_ns)/1e6)))
