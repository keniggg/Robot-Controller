"""The planning hint must name the exact failed terminal curve."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint

from test_gen52_contact_terminal_collision_screen import case, module


@pytest.mark.parametrize('fault',[None,'not_terminal','changed_q','other_curve','other_stage'])
def test_compact_diagnostic_only_for_exact_failed_compensated_terminal(monkeypatch,fault):
    data,fk,sample,nominal,scene,g,goal,_=case(0)
    curve,_=module.predict_contact_command_path(nominal,fk,sample,goal)
    with pytest.raises(module.ObservationPathError) as caught:
        module.contact_curve_terminal_collision(curve,fk,scene,g,data['opening_m'])
    error=caught.value
    if fault=='not_terminal':error.contact_replay_sample['endpoint_screen_only']=False
    if fault=='changed_q':error.contact_replay_sample['joint_positions_rad'][0]+=.001
    audit={};stored=[]
    monkeypatch.setattr(module._ContactReplayLogStore,'put',lambda _,value:stored.append(value) or {'diagnostic_only':True})
    plan=NS(plan_id=scene.plan_id,poses=[deepcopy(goal) for _ in range(4)],
        contact_geometry_evidence='',  # Receipt adapter fixture, never a signed executable plan.
        diagnostic='CONTACT_EXECUTION_PLAN',model_choice='unknown_tabletop')
    sink=module._contact_replay_failure_sink(audit,plan,scene,data['model']['xml'],sample['epoch_ns'],
        {'mode':'unknown','generation':1},sample,{}, {}, {},g,data['opening_m'],'current_accepted_open_posture')
    kind='nominal' if fault=='other_curve' else 'predicted_command'
    stage='approach' if fault=='other_stage' else 'grasp'
    hold=JointTrajectoryPoint(positions=list(curve.joint_trajectory.points[0].positions),
        velocities=[0.]*6,accelerations=[0.]*6)
    module._emit_contact_replay_failure(sink,error,curve,hold,kind,stage)
    if fault is not None:
        assert 'contact_terminal_failure' not in audit
    else:
        report=audit['contact_terminal_failure']
        assert report['plan_id']==scene.plan_id and report['source_stamp_ns']==scene.source_ns
        assert report['model_sha256']==fk.model_sha256
        assert report['trajectory_sha256']==module.wire_digest(curve)
        np.testing.assert_array_equal(report['command_endpoint_rad'],curve.joint_trajectory.points[-1].positions)
        assert report['execution_authorized'] is False and report['endpoint_screen_only'] is True
    assert stored and audit['contact_failure_replay']['status']=='stored'
