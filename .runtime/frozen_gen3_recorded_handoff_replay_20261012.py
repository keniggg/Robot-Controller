"""Offline replay of archived GEN3 notices; does not initialize/use ROS nodes."""
from pathlib import Path
import hashlib
import importlib.util
import json
import rosbag
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan

WORK=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=BASE/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen3_analysis'
RECORD=BASE/'grasp_records/20261012T032110.570728Z_a1a1c737f44d453aa25a5585504da760'
SOURCE=WORK/'.runtime/gen3_contact_execution_handoff_20261012/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/frozen_contact_handoff.py'
ISSUED_NS=1791775378250409000
def sha(raw): return hashlib.sha256(raw).hexdigest()
source_hash=sha(SOURCE.read_bytes())
spec=importlib.util.spec_from_file_location('offline_recorded_frozen_contact_handoff',str(SOURCE))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
raw=(OUT/'accepted_preview.rosmsg').read_bytes()
plan=Grasp6DPlan().deserialize(raw)
session=None; phase=None; epoch=None
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for topic,message,stamp in bag.read_messages(topics=['/grasp_mode/observe_return_session','/grasp/near_field_phase','/alicia_d/control_reference_epoch']):
        if stamp.to_nsec()>ISSUED_NS: continue
        if topic=='/grasp_mode/observe_return_session':
            session=dict(received_ns=stamp.to_nsec(),raw_json=message.data,value=json.loads(message.data))
        elif topic=='/grasp/near_field_phase':
            phase=dict(received_ns=stamp.to_nsec(),source_ns=message.header.stamp.to_nsec(),
                active=message.active,phase_id=message.phase_id,deadline_ns=message.deadline.to_nsec(),
                reference_target_track_id=message.reference_target_track_id,reference_source_ns=message.reference_source_stamp.to_nsec(),planning_stage=message.planning_stage)
        else:
            epoch=dict(received_ns=stamp.to_nsec(),epoch_ns=message.stamp.to_nsec())
assert session is not None and phase is not None and epoch is not None
assert phase['active'] is True and phase['reference_target_track_id']==plan.target_track_id
alignment=json.loads((RECORD/'alignment_receipt.json').read_text())
assert alignment['epoch_ns']==epoch['epoch_ns']
inputs=dict(selection=session['value']['selection'],session=session['value'],epoch_ns=alignment['epoch_ns'],
    issued_ns=ISSUED_NS,phase_id=phase['phase_id'],phase_deadline_ns=phase['deadline_ns'])
output=module.build_contact_freeze_notice(plan,**inputs)
encoded=json.dumps(output,sort_keys=True,separators=(',',':'),allow_nan=False)
parsed=module.parse_contact_freeze_notice(encoded)
assert parsed==output
assert output['source_ns']==plan.header.stamp.to_nsec()
assert sha(SOURCE.read_bytes())==source_hash, 'staging source changed during replay'
report=dict(scope='Offline actual GEN3 preview/session/phase integer/string timestamp encoding replay only; not gateway receipt acceptance, motion or hardware validation',
    build_and_parse_accepted=True,source=dict(path=str(SOURCE),sha256=source_hash),
    preview=dict(path=str(OUT/'accepted_preview.rosmsg'),sha256=sha(raw),plan_id=plan.plan_id,source_ns=plan.header.stamp.to_nsec(),fused_view_count=plan.fused_view_count,refinement_status=plan.refinement_status),
    recorded_session=session,recorded_phase=phase,recorded_epoch=epoch,
    alignment_epoch=dict(path=str(RECORD/'alignment_receipt.json'),sha256=sha((RECORD/'alignment_receipt.json').read_bytes()),epoch_ns=alignment['epoch_ns'],
        selection_from_alignment_ignored=True,reason='Alignment receipt carries an earlier generation; selection is taken from actual recorded current observe-return session.'),
    inputs=inputs,issued_ns_provenance='Parent-supplied original task-freeze timestamp approximation; historical 1791775378250409000 ns, not current wall clock.',
    output=output,encoded_json=encoded,encoded_sha256=sha(encoded.encode()),roundtrip_equal=True,
    limits=['No ROS node/service/topic call or motion was performed.','No gateway frozen-plan payload, current control-owner, freshness or trajectory acceptance was exercised.','Current staging parser accepted historical record only; this is not authorization to reuse the expired plan now.'])
target=OUT/'recorded_frozen_handoff_replay.json'
target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(target),sha256=sha(target.read_bytes()),source_sha256=source_hash,plan_id=plan.plan_id,accepted=True,phase_deadline_ns=phase['deadline_ns']),ensure_ascii=False))
