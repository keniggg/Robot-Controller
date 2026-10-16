#!/usr/bin/env python3
"""Offline static workspace samples; no ROS I/O or execution authority.

These are joint-limit-valid FK configurations with a fixed recorded support
plane. They do not prove full-arm collision, IK search completeness, a path,
visibility, contact feasibility, hardware following, or grasp success.
"""
import argparse
import itertools
import json
from collections import Counter
from pathlib import Path

import numpy as np
from alicia_flexible_grasp.robot.observation_path_guard import (
    SerialUrdfFk, ObservationPathError, observation_following_support_bound,
    observation_tracking_error_bounds,
)
from alicia_flexible_grasp.robot.observation_tracking_contract import tracking_contract_candidates
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    fixture = root/'src/alicia_flexible_grasp_supervisor/tests/fixtures/observation_contract_20260919.json'
    data = json.loads(fixture.read_text())
    model = SerialUrdfFk((root/'src/real-arm/alicia_d_descriptions/urdf/alicia_duo_with_gripper.urdf').read_text(),
                         ['Joint%d'%i for i in range(1,7)])
    cfg = {n: {'trajectory': .12, 'goal': .035} for n in model.names}
    gripper = GripperGeometry(.05, .002, [.0434,.0286,.060], [.1175,.155,.0774], .003)
    geom, rows = data['geometry'], []
    choices = tracking_contract_candidates(cfg, model.names, .5)
    for yaw, shoulder, elbow in itertools.product((-.1,.7,1.4,2.1), (-.3,0.,.3), (-.3,0.,.3)):
        q = np.array(data['start_q'])+np.array([yaw,shoulder,elbow,0.,0.,0.])
        row = dict(joints_rad=q.tolist(), deltas_rad=[yaw,shoulder,elbow], trials=[])
        try:
            transform = model(q)
            row['tool_position_m'] = transform[:3,3].tolist()
            bands = [('configured', observation_tracking_error_bounds(cfg,model.names))]
            bands += [(str(c['path_position_tolerance_rad'][0]), c['joint_error_bounds_rad']) for c in choices]
            for label, errors in bands:
                try:
                    report = observation_following_support_bound(model,q,errors,geom['support_normal_base'],
                        geom['support_offset_m'],gripper,data['opening_m'],max_checks=2048,max_seconds=.5)
                    trial = dict(contract=label, ok=bool(report['ok']),
                        nominal_m=report['nominal_minimum_support_clearance_m'],
                        bound_m=report.get('minimum_support_clearance_lower_bound_m'),
                        reason=report.get('reason',''))
                except ObservationPathError as exc:
                    trial = dict(contract=label, ok=False, unresolved=True, reason=str(exc))
                row['trials'].append(trial)
            nominal = next((r['nominal_m'] for r in row['trials'] if 'nominal_m' in r), None)
            row['status'] = ('nominal_support_collision' if nominal is not None and nominal < .003 else
                'configured_box_pass' if row['trials'][0]['ok'] else
                'per_goal_conditional_box_pass' if any(t['ok'] for t in row['trials'][1:]) else
                'unresolved_or_conditional_box_rejected')
        except ObservationPathError as exc:
            row.update(status='joint_limit_or_input_rejected', reason=str(exc))
        rows.append(row)
    result = dict(scope=__doc__, model_sha256=model.model_sha256, fixed_plane=geom,
                  summary=dict(Counter(r['status'] for r in rows)), samples=rows)
    out = Path(args.output)
    with out.open('x') as f: json.dump(result,f,indent=2); f.write('\n')
    print(json.dumps(result['summary'],sort_keys=True))


if __name__ == '__main__': main()
