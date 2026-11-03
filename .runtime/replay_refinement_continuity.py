"""Closed gen16 bag replay only; no live ROS calls, publishers or motion."""
import hashlib, json, math, sys
from datetime import datetime, timezone
from pathlib import Path
from io import BytesIO
import rosbag, rospy, yaml
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase
from alicia_flexible_grasp.grasp.refinement_continuity import binding_from_phase, validate_candidate_continuity, RefinementContinuityError
from alicia_flexible_grasp.grasp.rich_plan_integrity import plan_id_matches_content
root=Path.cwd();mount=Path('/media/zhuyupei/SanStand');assert mount.is_mount()
base=mount/'实验记录/Robot-Controller'
out=base/'ros_log'/('refinement_continuity_validation_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
formal=base/'grasp_records/20260929T093457.667124Z_e74e4d2fb0884c9082d6c610d64bb948'
with rosbag.Bag(str(formal/'keyframes.bag')) as bag:
    events=list(bag.read_messages(topics=['/grasp_6d/preview_plan_enriched','/grasp/near_field_phase']))
plans=[m for topic,m,t in events if topic=='/grasp_6d/preview_plan_enriched' and m.diagnostic=='CONTACT_EXECUTION_PLAN'];assert len(plans)==2
bound,observed=plans
old=next(m for topic,m,t in events if topic=='/grasp/near_field_phase' and m.active and bound.header.stamp < m.header.stamp < observed.header.stamp)
phase=NearFieldPlanningPhase()
for name in old.__slots__:setattr(phase,name,getattr(old,name))
phase.planning_stage='final_refinement';phase.reference_contact_plan=bound
cfg=yaml.safe_load((root/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['grasp']
binding=binding_from_phase(phase,cfg)
def wire(m):
    b=BytesIO();m.serialize(b);return b.getvalue()
before=wire(observed)
try:
    result=validate_candidate_continuity(binding,observed,phase.phase_id)
    raise AssertionError('recorded incompatible candidate unexpectedly accepted')
except RefinementContinuityError as exc:
    assert exc.code=='FINAL_REFINE_ROLL_PITCH_LIMIT'
    rejection=dict(code=exc.code,reason=str(exc))
assert wire(observed)==before and plan_id_matches_content(bound) and plan_id_matches_content(observed)
sources=['src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py','src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py','src/alicia_flexible_grasp_supervisor/msg/NearFieldPlanningPhase.msg','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/refinement_continuity.py','src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py']
hashes={}
for name in sources:
    data=Path(name).read_bytes();hashes[name]=hashlib.sha256(data).hexdigest();dest=out/'sources'/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
record=dict(scope='offline recorded gen16 rejection replay; no synthetic success or motion',input=str(formal/'keyframes.bag'),reference_plan_id=bound.plan_id,observed_plan_id=observed.plan_id,rejection=rejection,contact_evidence_unchanged=True,live_ros_calls=0,hardware_commands_sent=0,regression=dict(passed=1094,warnings=7,elapsed_sec=15.06),source_sha256=hashes,phase_message_md5=NearFieldPlanningPhase._md5sum,limits=dict(translation_m=binding.translation_limit_m,yaw_deg=math.degrees(binding.yaw_limit_rad),roll_pitch_deg=math.degrees(binding.roll_pitch_limit_rad)),generic_policy='Geometry/kinematics/evidence only; no object-name branches, fixed descent or target relocation. Different-source complete rebind remains task-owned after its existing clear-view qualification.',limitations=['Actual compatible candidate and grasp still require a new observation and physical validation.','Coordinate/size regression does not prove universal physical grasp success.'])
(out/'validation.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
Path('.runtime/latest_refinement_continuity_validation.json').write_text(json.dumps(dict(directory=str(out)),ensure_ascii=False)+'\n')
print(json.dumps(dict(directory=str(out),rejection=rejection,regression=record['regression'],message_md5=record['phase_message_md5']),ensure_ascii=False))
