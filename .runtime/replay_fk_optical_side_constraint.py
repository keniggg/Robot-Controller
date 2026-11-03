#!/usr/bin/env python3
"""Sealed-source FK proposal replay; no ROS services or motion authority."""
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_grasp_modes.reachable_observation_views import generate_reachable_observation_views

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z')
OUT = ROOT/'verification/fk_optical_side_evidence'


def inputs(source_root):
    source = json.loads((source_root/'source.json').read_text())
    with np.load(str(source_root/'source.npz')) as arrays:
        source.update({key: arrays[key].copy() for key in arrays.files})
    provenance = source['source_provenance']
    raw = Path(provenance['audit_path']).read_bytes()
    audit = json.loads(raw)
    assert hashlib.sha256(raw).hexdigest() == provenance['audit_sha256']
    original = audit['observation_failure_diagnostics']['joint_space_fk_search']['audit']
    source['geometry'] = audit['replay_geometry']
    fk = SerialUrdfFk(Path(provenance['model_path']).read_text(), ['Joint%d' % i for i in range(1, 7)])
    assert fk.model_sha256 == original['model_sha256']
    return dict(source=source, current_joint_positions=original['current_joint_positions'], fk=fk,
        gripper=GripperGeometry(.05, .002, np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003),
        opening_width_m=.05, gaps=audit['observe_return']['measured_missing_sides'],
        depth_uncertainty_m=.003, side_depth_uncertainty_m=.003,
        joint_error_bounds_rad=original['joint_error_bounds_rad'], max_seconds=2., max_directions=2,
        proposal_image_reserve_px=4.)


def main():
    spec = importlib.util.spec_from_file_location('alicia_grasp_modes.before_fk_optical_side', str(OUT/'before_reachable_observation_views.py'))
    baseline = importlib.util.module_from_spec(spec); spec.loader.exec_module(baseline)
    for label, path in [('gen14', ROOT/'gen14_frozen_source'), ('gen10', ROOT/'gen10_offline_ik/frozen_source')]:
        arguments = inputs(path)
        cases = []
        # New bounded production search first; baseline is a separate inert comparison.
        for name, function in [('current', generate_reachable_observation_views),
                               ('baseline', baseline.generate_reachable_observation_views)]:
            kwargs = dict(arguments)
            if name == 'baseline': kwargs.pop('side_depth_uncertainty_m')
            candidates, audit = function(**kwargs)
            case = dict(implementation=name, candidates=candidates, audit=audit)
            cases.append(case)
            print(json.dumps(dict(record=label, implementation=name, count=len(candidates),
                status=audit['status'], elapsed_sec=audit['elapsed_sec'],
                directions=audit['directions'])), flush=True)
            (OUT/('%s_%s.json' % (label, name))).write_text(json.dumps(case, indent=2, allow_nan=False)+'\n')
            if name == 'current':
                for index, candidate in enumerate(candidates):
                    (OUT/('%s_candidate%d.json' % (label, index))).write_text(
                        json.dumps(candidate, indent=2, allow_nan=False)+'\n')
        result = dict(record=label, source_json_sha256=hashlib.sha256((path/'source.json').read_bytes()).hexdigest(),
            source_npz_sha256=hashlib.sha256((path/'source.npz').read_bytes()).hexdigest(),
            model_sha256=arguments['fk'].model_sha256, results=cases, ros_calls=False,
            execution_authority=False, whole_arm_path_checked=False,
            source_limitations=arguments['source'].get('limitations', []),
            depth_uncertainty_m=.003, side_depth_uncertainty_m=.003)
        (OUT/(label+'_replay.json')).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__': main()
