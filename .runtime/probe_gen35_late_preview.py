from copy import deepcopy
import json, hashlib, os
from pathlib import Path
from _pytest.monkeypatch import MonkeyPatch
from test_task_modes import configured
from test_station_compute_wait_task import station, frame, deliver, advance, feedback, at_ns, base, recorded

mp = MonkeyPatch()
paths = [Path('src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'),
    Path('src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py'),
    Path('src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py')]
hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
c = station.__wrapped__(configured.__wrapped__(mp), mp)
deliver(c, frame(c))
old = deepcopy(c.node._station_task_record['contract'])
advance(c, 100_900_000_000)
c.now[0] = 101_050_000_000
feedback(c, 100_980_000_000)
feedback(c, 101_020_000_000)
deliver(c, frame(c, 101_000_000_000, old))
_, _, contact, _, _ = recorded(2)
contact.header.stamp = at_ns(101_000_000_000)
contact.plan_id = base.compute_plan_id(contact)
c.node.grasp6d_preview_plan_cb(contact)
assert c.node._station_contact_proposal is not None
def state():
    return dict(retained_plan_id=getattr(c.node._station_contact_proposal, 'plan_id', None),
        public_valid=c.node.latest_grasp6d_preview_plan.valid,
        source_ns=c.node.latest_grasp6d_preview_plan.header.stamp.to_nsec(),
        station_pending=c.node._station_task_record['pending'], station_retired=c.node._station_task_retired)
before = state()
oldpreview = base.Grasp6DPlan()
oldpreview.header.stamp = at_ns(100_900_000_000)
oldpreview.header.frame_id = 'base_link'
oldpreview.valid = False
oldpreview.candidate_source = 'near_field_pending'
oldpreview.target_track_id = contact.target_track_id
oldpreview.diagnostic = 'TARGET_PROCESSING_PENDING: '+json.dumps(dict(
    station_compute_wait=old, station_compute_recovery_chain=None, clock_only=True),
    sort_keys=True, separators=(',', ':'))
c.node.grasp6d_preview_plan_cb(oldpreview)
after = state()
report = dict(kind='offline_real_task_callback_late_clock_preview', before=before, after=after,
    reproduced=before['retained_plan_id'] is not None and after['retained_plan_id'] is None and after['public_valid'] is False,
    new_registered_source_ns=c.node._station_task_record['contract']['source_ns'],
    old_preview_source_ns=100_900_000_000, source_sha256=hashes,
    source_unchanged=all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in paths),
    live_ros=False, hardware_actions=0,
    limitation='Existing inert transport station fixture; callback ordering regression, not asserted as cause of GEN33 recorded physical aperture rejection. Contact preview is computational work, not execution authority.')
mp.undo()
Path(os.environ['GEN35_PROBE_OUTPUT'], 'result.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
