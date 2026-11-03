#!/usr/bin/env python3
"""Bounded GEN33 recorded geometry -> current candidate rejection, offline.

This is not a whole-task replay. No historical model digest, registration,
source admission, or physical result is synthesized from the saved audit.
"""
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import asdict
from collections import Counter
import argparse
import hashlib
import importlib.util
import json
import os
import socket
import sys
import time
import traceback

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
C = E / 'ros_log/full_latest_gen29_restart_20261016T000728Z'
R = E / 'grasp_records/20261016T103326.181919Z_515e06336cbb4a4799e6adfb9277b5cb'
D = C / 'verification/gen33_attempt1_geometry_diagnosis_r1'
AUDIT_SHA = 'acd1f9119daa4ccb3a1ecdf950c182dfe860963d4b09e6b8e86963b72b0e6b48'
RECON_SHA = '600c0761806fb8148b78bad2ce391d50fecb580fc968f34e66d22f8051d57dc1'
INVENTORY_SHA = 'd1ba47caa4a7dc1a5ebb132c4df02e15e68921c32453316d009f1baa52c2f62f'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(part)
    return h.hexdigest()


class ExternalBoundaryViolation(BaseException):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand unavailable'
    assert out.parent == (C / 'verification').resolve()
    assert out.name.startswith('gen35_recorded_failure_replay_')
    out.mkdir(exist_ok=False)
    (out / 'tmp').mkdir()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(out / 'ros'),
        TMPDIR=str(out / 'tmp'), PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1')
    sys.dont_write_bytecode = True
    boundary_calls = []

    def forbidden(*args, **kwargs):
        boundary_calls.append(repr(args)[:500])
        raise ExternalBoundaryViolation('Unsimulated external boundary')

    # BaseException intentionally escapes broad production Exception catches.
    socket.socket.connect = forbidden
    socket.socket.connect_ex = forbidden
    socket.socket.sendto = forbidden
    socket.create_connection = forbidden
    socket.getaddrinfo = forbidden
    for item in ('src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
                 'devel/lib/python3/dist-packages'):
        sys.path.insert(0, str(W / item))
    sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
    report = dict(kind='gen35_recorded_failure_current_candidate_replay', passed=False,
                  physical_grasp_success=False, whole_task_end_to_end=False,
                  historical_callback_bug_cause_claimed=False, live_ros_contacted=False)
    before = {}
    try:
        import numpy as np
        import yaml
        import rospy
        for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                     'wait_for_service', 'get_master', 'get_param_cached', 'set_param', 'delete_param'):
            setattr(rospy, name, forbidden)
        rospy.get_param = forbidden
        # Import code only; no node constructor or worker is run.
        source = W / 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
        spec = importlib.util.spec_from_file_location('gen35_recorded_failure_remote', source)
        remote = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(remote)
        from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference, enclose_inferred_box_geometry
        from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
        from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
        from alicia_flexible_grasp.vision.multiview_surface import SurfaceView, FusedTargetSurface
        from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry

        audit_path = R / ('keyframes_audits/' + AUDIT_SHA + '.json')
        reconstruction_path = D / 'contact_model_reconstruction.json'
        inventory_path = D / 'replay_inputs.json'
        assert sha(audit_path) == AUDIT_SHA
        assert sha(reconstruction_path) == RECON_SHA
        assert sha(inventory_path) == INVENTORY_SHA
        manifest_path = R / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
        for row in manifest['files']:
            if row['path'] in ('keyframes.bag', str(audit_path.relative_to(R))):
                assert sha(R / row['path']) == row['sha256']
        audit = json.loads(audit_path.read_text())
        data = json.loads(reconstruction_path.read_text())
        inv = json.loads(inventory_path.read_text())
        assert data['qualified'] is True and data['model_sha256_exactly_matches_recorded'] is False
        assert data['source_stamp_ns'] == audit['snapshot_stamp_ns'] == 1792146855672073125
        assert data['inputs_sha256'][str(audit_path)] == AUDIT_SHA
        assert data['inputs_sha256'][str(inventory_path)] == INVENTORY_SHA
        table = audit['remote_diagnostics']['tabletop_geometry']
        model = RegularBoxInference.from_evidence_dict(data['model'])
        assert model.model_sha256 != table['regular_box_inference']['model_sha256']
        model_source = model.to_evidence_dict()['source']
        assert model_source['stamp_ns'] == audit['snapshot_stamp_ns']
        original = data['original_geometry']
        geometry = GeometryEstimate(True, '', '', np.array(original['center_base']),
            np.array(original['axes_base']), np.array(original['size_xyz_m']),
            np.array(original['support_normal_base']), original['support_offset_m'],
            original['support_inlier_ratio'], np.array(original['object_points_base']), original['source_mode'])
        points_hash = lambda: hashlib.sha256(np.ascontiguousarray(geometry.object_points_base).tobytes()).hexdigest()
        assert points_hash() == audit['replay_geometry']['object_points_sha256']
        assert len(geometry.object_points_base) == 2265
        planning, enclosure = enclose_inferred_box_geometry(geometry, model)
        historical_enclosure = table['regular_box_planning_enclosure']
        assert np.max(np.abs(planning.size_xyz_m - historical_enclosure['planning_size_xyz_m'])) < 1e-8

        config_path = W / 'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
        baseline_path = W / '.runtime/gen34_baseline_source/manifest.json'
        baseline = json.loads(baseline_path.read_text())
        assert sha(config_path) == baseline['source_sha256'][str(config_path.relative_to(W))]
        config = yaml.safe_load(config_path.read_text())
        cfg = config['grasp_6d']['remote']
        gripper = cfg['gripper_geometry']
        node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
        node.gripper_geometry = GripperGeometry(gripper['max_inner_gap_m'],
            gripper['width_safety_margin_per_side_m'], np.array(gripper['finger_box_xyz_m']),
            np.array(gripper['palm_box_xyz_m']), gripper.get('support_clearance_m', .003))
        node.opening_fit_clearance_each_side_m = remote.load_opening_fit_clearance_config(gripper, node.gripper_geometry)
        node.tabletop_geometry_enabled, node.tabletop_geometry_config, node.hybrid_merge_config = remote.load_tabletop_geometry_config(
            cfg, node.gripper_geometry, node.opening_fit_clearance_each_side_m)
        node.adaptive_stage_limits = remote.load_adaptive_stage_config(cfg)
        node.gripper_tool_jaw_axis = gripper['tool_jaw_axis']
        node.gripper_tool_finger_length_axis = gripper['tool_finger_length_axis']
        node.candidate_min_downward_approach_cos = cfg['candidate_min_downward_approach_cos']
        node.candidate_max_final_approach_lateral_m = cfg['candidate_max_final_approach_lateral_m']
        node.near_field_planning_active = True
        stamp = audit['snapshot_stamp_ns']
        error = table['adaptive_stage_profiles'][0]['execution_position_error_m']
        parameter_reads = []
        def recorded_parameter(name, default=None):
            if name != '/grasp_6d/runtime_execution_error':
                return forbidden(name)
            parameter_reads.append(name)
            return dict(position_error_m=error, stamp_sec=stamp / 1e9)
        rospy.get_param = recorded_parameter
        rospy.Time.now = staticmethod(lambda: rospy.Time(*divmod(stamp, 10**9)))
        identity = TargetTrackIdentity(**model_source['target_identity'])
        view = SurfaceView(identity, stamp, geometry.object_points_base, geometry.support_normal_base,
                           geometry.support_offset_m, 12)
        # A diagnostic carrier for already saved points, not reconstructed
        # historic per-voxel view lineage. The inferred proposal branch is used.
        surface = FusedTargetSurface(identity, geometry.object_points_base,
            np.zeros(len(geometry.object_points_base), dtype=np.int64), (stamp,))
        node._snapshot_multiview_surface = lambda: (surface, view)
        node._current_inferred_box = lambda: model
        node._regular_box_snapshot = dict(result=NS(audit=data['audit'], ok=True, code=data['code']), enclosure=enclosure)
        snapshot = NS(stamp_ns=stamp, target_identity=identity, quality=NS(
            depth_mad_m=table['depth_spatial_mad_m'], depth_repeatability_m=table['depth_repeatability_m']))
        assert node.opening_fit_clearance_each_side_m == .0005
        assert node.gripper_geometry.max_inner_gap_m == .05
        dependencies = {source, config_path, baseline_path, Path(__file__).resolve()}
        for module in list(sys.modules.values()):
            filename = getattr(module, '__file__', None)
            if filename and str(filename).startswith(str(W / 'src')) and Path(filename).suffix == '.py':
                dependencies.add(Path(filename).resolve())
        before = {str(path): sha(path) for path in sorted(dependencies)}
        trace = Counter()
        interesting = {'_generate_tabletop_candidates', 'generate_tabletop_proposals', '_generate_inferred_proposals',
                       'evaluate_inferred_box_contact', '_obb_line_interval', 'materialize_tabletop_candidates'}
        def profile(frame, event, arg):
            if event == 'call' and frame.f_code.co_name in interesting:
                trace[str(Path(frame.f_code.co_filename).resolve()) + ':' + frame.f_code.co_name] += 1
        sys.setprofile(profile)
        started, cpu_started = time.perf_counter(), time.process_time()
        try:
            candidates, diagnostics = node._generate_tabletop_candidates(planning, snapshot)
        finally:
            sys.setprofile(None)
        elapsed, cpu = time.perf_counter() - started, time.process_time() - cpu_started
        assert not candidates and diagnostics['proposal_count'] == 1 and diagnostics['materialized_count'] == 0
        assert diagnostics['failure_code'] == 'GRIPPER_SWEEP_COLLISION'
        rejection, = diagnostics['proposal_reach_rejections']
        old_rejection, = table['proposal_reach_rejections']
        assert rejection['failed_gate'] == 'finger_reach'
        for key in ('negative_reach_m', 'positive_reach_m', 'maximum_side_reach_m'):
            assert abs(rejection[key] - old_rejection[key]) < 1e-8
        total = rejection['negative_reach_m'] + rejection['positive_reach_m']
        assert total > .05 and total / 2 > rejection['maximum_side_reach_m']
        assert points_hash() == audit['replay_geometry']['object_points_sha256']
        assert boundary_calls == []
        assert any(key.endswith(':_generate_inferred_proposals') for key in trace)
        assert not any(key.endswith(':materialize_tabletop_candidates') for key in trace)
        after = {path: sha(path) for path in before}
        assert before == after
        report.update(passed=True, source_stamp_ns=stamp, input_sha256={str(p):sha(p) for p in
            (manifest_path, audit_path, reconstruction_path, inventory_path)}, source_before=before,
            source_after=after, source_unchanged=True, config=asdict(node.tabletop_geometry_config),
            archived_model_sha256=table['regular_box_inference']['model_sha256'],
            independently_recomputed_model_sha256=model.model_sha256, exact_historical_model=False,
            measured_point_count=2265, measured_points_sha256=points_hash(), measured_points_unchanged=True,
            original_enclosure=historical_enclosure, replay_enclosure=enclosure, actual_diagnostics=diagnostics,
            historical_reach=old_rejection, total_required_reach_m=total,
            balanced_side_reach_m=total/2, trace=dict(trace), wall_sec=elapsed, cpu_sec=cpu,
            external_boundary_attempts=boundary_calls, simulated_parameter_reads=parameter_reads,
            action_calls=0, materialization_calls=0, source_admission_replayed=False,
            limitations=[
                'Calls the current candidate generator and unchanged inferred proposal/finger reach rejection, not the full task FSM or learned candidates.',
                'The saved same-source recomputed canonical model is independently validated; its SHA differs from the historical model and is never substituted into the historical audit.',
                'The diagnostic surface carrier contains the exact archived 2265 measured points but does not claim historic per-voxel view lineage or recreate registration.',
                'Live model/surface accessors are replaced only by frozen source providers; production source admission is outside this bounded replay.',
                'Sealed GEN33 source YAML supplies geometry settings; the whole runtime remote configuration was not recorded. Exact aperture/fit and resulting recorded reach are checked.',
                'No motion service is called. The original experiment had one completed observation; this replay proves zero new action, not zero historical motion.',
                'GEN35 callback ordering bugs are independently reproduced defects and are not asserted as the cause of this recorded physical aperture failure.'])
    except BaseException as exc:
        sys.setprofile(None)
        report.update(error=repr(exc), traceback=traceback.format_exc(), source_before=before,
                      external_boundary_attempts=boundary_calls)
    (out / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(passed=report['passed'], result=str(out / 'result.json'),
                         sha256=sha(out / 'result.json'), error=report.get('error'))))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
