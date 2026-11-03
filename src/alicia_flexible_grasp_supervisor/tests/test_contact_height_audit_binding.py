"""Height evidence must survive exact JSON audit promotion, including tamper checks."""
import json
from pathlib import Path
from test_contact_height_placement import case
from test_remote_grasp6d_streaming import (promotion_node, promotion_plan,
    cached_preview_audit_report, RecordingPublisher, PromotionDecision, remote_node)
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height


def test_height_evidence_has_identical_in_memory_and_json_values():
    transform,audit=place_contact_height(translation_evidence_limit_m=.001,**case())
    assert transform is not None
    assert audit==json.loads(json.dumps(audit,allow_nan=False))


def test_height_evidence_can_promote_but_changed_disk_evidence_cannot(tmp_path):
    _,audit=place_contact_height(translation_evidence_limit_m=.001,**case())
    node=promotion_node();node.gate_audit_enabled=True
    node.gate_audit_output_path=str(tmp_path/'preview.json')
    node.mujoco_audit_output_path=str(tmp_path/'mujoco.json')
    node.gate_audit_pub=RecordingPublisher()
    node._execution_promotion_audit_ready=remote_node.RemoteGrasp6DNode._execution_promotion_audit_ready.__get__(node)
    report=cached_preview_audit_report('plan-B')
    report['selected']['contact_height_placement']=audit
    report['selected']['final_contact_height_margin']=audit['after']
    node._active_gate_audit_report=report
    plan=promotion_plan('plan-B')
    ready,reason=node._write_cached_preview_execution_audit(plan,
        PromotionDecision(True,'PROMOTE_REPLAN','explicit execution replan requested'))
    assert ready and not reason
    path=Path(node._latest_execution_audit_reference['report_path'])
    saved=json.loads(path.read_text())
    assert saved==node._active_execution_audit_report
    assert node._execution_promotion_audit_ready(plan)
    saved['selected']['contact_height_placement']['normal_translation_m']+=.001
    path.write_text(json.dumps(saved,indent=2,sort_keys=True))
    assert not node._execution_promotion_audit_ready(plan)
