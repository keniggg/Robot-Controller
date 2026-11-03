#!/usr/bin/env python3
"""Two closed recordings through current candidate/contact gates, no live IO.

The GEN35 box is the exact previously pixel-recomputed historical model.
The GEN33 box is a recorded same-source canonical reconstruction with a
different digest. Neither frozen carrier recreates registration/admission.
"""
from pathlib import Path
from types import SimpleNamespace as NS
from collections import Counter
from dataclasses import asdict
import argparse
import builtins
import hashlib
import importlib.util
import json
import os
import platform
import socket
import subprocess
import sys
import time
import traceback

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
C = E / 'ros_log/full_latest_gen29_restart_20261016T000728Z'
V = C / 'verification'
R33 = E / 'grasp_records/20261016T103326.181919Z_515e06336cbb4a4799e6adfb9277b5cb'
R35 = E / 'grasp_records/20261016T121453.757769Z_87f1feb8ab9847419f85b9ab3d7237fb'
A33 = 'acd1f9119daa4ccb3a1ecdf950c182dfe860963d4b09e6b8e86963b72b0e6b48'
A35 = '1a2bda4538b3516d34934347bace57621290bccc6b808aef779b2bd19d3d0919'
PINNED = {
    V/'gen33_attempt1_geometry_diagnosis_r1/contact_model_reconstruction.json': '600c0761806fb8148b78bad2ce391d50fecb580fc968f34e66d22f8051d57dc1',
    V/'gen33_attempt1_geometry_diagnosis_r1/replay_inputs.json': 'd1ba47caa4a7dc1a5ebb132c4df02e15e68921c32453316d009f1baa52c2f62f',
    V/'gen35_actual_attempt1_box_routes_r2/result.json': 'bd2a9e45e983fe338150deba339e466c83c07ea83384b25214527945bf40a89d',
    V/'gen35_recorded_failure_replay_r2/result.json': 'bee4bd50779c5ae6d28a1af913f9cab68d422562f6fe5496bb413bc1104eb8e6',
}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for part in iter(lambda: stream.read(1024*1024), b''):
            h.update(part)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


class ExternalBoundaryViolation(BaseException):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand unavailable'
    assert out.parent == V.resolve() and out.name.startswith('gen36_recorded_negative_')
    out.mkdir(exist_ok=False)
    (out/'tmp').mkdir()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(out/'ros'),
        TMPDIR=str(out/'tmp'), PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1')
    sys.dont_write_bytecode = True
    calls = []
    # Python 3.8 platform.uname otherwise forks `uname -p` during ROS import.
    # Populate its environmental cache directly from the local uname syscall;
    # keep the process guard unconditional for project code and other imports.
    uname = os.uname()
    platform._uname_cache = platform.uname_result(uname.sysname, uname.nodename,
        uname.release, uname.version, uname.machine, uname.machine)

    def forbidden(*args, **kwargs):
        calls.append(repr(args)[:300])
        raise ExternalBoundaryViolation('Unsimulated external boundary')

    # Guards installed before importing project/ROS code. BaseException escapes
    # production broad Exception catches and makes an accidental boundary fail.
    for name in ('connect', 'connect_ex', 'sendto', 'sendmsg', 'bind', 'listen'):
        if hasattr(socket.socket, name):
            setattr(socket.socket, name, forbidden)
    socket.create_connection = forbidden
    socket.getaddrinfo = forbidden
    subprocess.Popen = forbidden

    def io_audit(event, args):
        if event == 'open' and args and isinstance(args[0], (str, bytes)):
            path = os.fsdecode(args[0])
            if path.startswith(('/dev/ttyUSB', '/dev/ttyACM', '/dev/video', '/dev/bus/usb')):
                forbidden('device open', path)
        if event in ('subprocess.Popen', 'os.system', 'os.exec', 'os.posix_spawn'):
            forbidden(event)
    sys.addaudithook(io_audit)
    for item in ('src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
                 'devel/lib/python3/dist-packages'):
        sys.path.insert(0, str(W/item))
    sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
    report = dict(kind='gen36_current_candidate_recorded_negative_replay', passed=False,
        physical_grasp_success=False, whole_task_end_to_end=False,
        new_motion_commands_sent=0, live_ros_contacted=False,
        temperature_queried=False, production_modified=False)
    before = {}
    inputs = {}
    started = time.monotonic()
    try:
        import numpy as np
        import yaml
        import rospy
        for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                     'wait_for_service', 'get_master', 'get_param', 'get_param_cached',
                     'set_param', 'delete_param'):
            setattr(rospy, name, forbidden)
        source = W/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
        spec = importlib.util.spec_from_file_location('gen36_negative_remote', source)
        remote = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(remote)
        from alicia_flexible_grasp.vision.regular_box_inference import RegularBoxInference, enclose_inferred_box_geometry
        from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
        from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
        from alicia_flexible_grasp.vision.multiview_surface import SurfaceView, FusedTargetSurface
        from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry, evaluate_inferred_box_contact

        for path, digest in PINNED.items():
            assert sha(path) == digest, str(path)
            inputs[str(path)] = digest
        manifests = {}
        for record in (R33, R35):
            manifest = read(record/'manifest.json')
            assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
            inputs[str(record/'manifest.json')] = sha(record/'manifest.json')
            # Verify all files in each closed manifest, not just the audit used.
            for row in manifest['files']:
                path = (record/row['path']).resolve()
                path.relative_to(record.resolve())  # raises on escape; Python 3.8 compatible
                assert sha(path) == row['sha256'], str(path)
                inputs[str(path)] = row['sha256']
            manifests[str(record)] = dict(state=manifest['state'], recorder_closed=True,
                manifest_sha256=inputs[str(record/'manifest.json')], verified_files=len(manifest['files']))
        audit33 = read(R33/'keyframes_audits'/f'{A33}.json')
        audit35 = read(R35/'keyframes_audits'/f'{A35}.json')
        assert inputs[str(R33/'keyframes_audits'/f'{A33}.json')] == A33
        assert inputs[str(R35/'keyframes_audits'/f'{A35}.json')] == A35
        data33 = read(V/'gen33_attempt1_geometry_diagnosis_r1/contact_model_reconstruction.json')
        routes35 = read(V/'gen35_actual_attempt1_box_routes_r2/result.json')
        assert data33['qualified'] is True and data33['model_sha256_exactly_matches_recorded'] is False
        assert routes35['passed'] and routes35['exact_historical_model_digest'] is True
        for path, digest in routes35['inputs_sha256'].items():
            assert sha(path) == digest
            inputs[path] = digest
        config_path = W/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
        baseline_path = W/'.runtime/gen34_baseline_source/manifest.json'
        assert sha(config_path) == read(baseline_path)['source_sha256'][str(config_path.relative_to(W))]
        cfg = yaml.safe_load(config_path.read_text())['grasp_6d']['remote']
        dependencies = {source, config_path, baseline_path, Path(__file__).resolve()}
        for module in list(sys.modules.values()):
            filename = getattr(module, '__file__', None)
            if filename and str(filename).startswith(str(W/'src')) and Path(filename).suffix == '.py':
                dependencies.add(Path(filename).resolve())
        before = {str(path): sha(path) for path in sorted(dependencies)}

        def run_case(name, audit, evidence, original, exact):
            table = audit['remote_diagnostics']['tabletop_geometry']
            stamp = audit['snapshot_stamp_ns']
            model = RegularBoxInference.from_evidence_dict(evidence)
            archived = table['regular_box_inference']['model_sha256']
            assert (model.model_sha256 == archived) is exact
            assert model.to_evidence_dict()['source']['stamp_ns'] == stamp
            geometry = GeometryEstimate(True, '', '', np.asarray(original['center_base']),
                np.asarray(original['axes_base']), np.asarray(original['size_xyz_m']),
                np.asarray(original['support_normal_base']), original['support_offset_m'],
                original['support_inlier_ratio'], np.asarray(original['points']), original['source_mode'])
            point_hash = lambda: hashlib.sha256(np.ascontiguousarray(geometry.object_points_base).tobytes()).hexdigest()
            assert point_hash() == audit['replay_geometry']['object_points_sha256']
            planning, enclosure = enclose_inferred_box_geometry(geometry, model)
            assert np.max(np.abs(planning.size_xyz_m - table['regular_box_planning_enclosure']['planning_size_xyz_m'])) < 1e-8
            gripper = cfg['gripper_geometry']
            node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
            node.gripper_geometry = GripperGeometry(gripper['max_inner_gap_m'],
                gripper['width_safety_margin_per_side_m'], np.asarray(gripper['finger_box_xyz_m']),
                np.asarray(gripper['palm_box_xyz_m']), gripper.get('support_clearance_m', .003))
            node.opening_fit_clearance_each_side_m = remote.load_opening_fit_clearance_config(gripper, node.gripper_geometry)
            node.tabletop_geometry_enabled, node.tabletop_geometry_config, node.hybrid_merge_config = remote.load_tabletop_geometry_config(
                cfg, node.gripper_geometry, node.opening_fit_clearance_each_side_m)
            node.adaptive_stage_limits = remote.load_adaptive_stage_config(cfg)
            node.gripper_tool_jaw_axis = gripper['tool_jaw_axis']
            node.gripper_tool_finger_length_axis = gripper['tool_finger_length_axis']
            node.candidate_min_downward_approach_cos = cfg['candidate_min_downward_approach_cos']
            node.candidate_max_final_approach_lateral_m = cfg['candidate_max_final_approach_lateral_m']
            node.near_field_planning_active = True
            parameter_reads = []
            def parameter(name, default=None):
                if name != '/grasp_6d/runtime_execution_error' or not table['adaptive_stage_profiles']:
                    return forbidden('unrecorded parameter', name)
                parameter_reads.append(name)
                return dict(position_error_m=table['adaptive_stage_profiles'][0]['execution_position_error_m'], stamp_sec=stamp/1e9)
            rospy.get_param = parameter
            rospy.Time.now = staticmethod(lambda: rospy.Time(*divmod(stamp, 10**9)))
            identity = TargetTrackIdentity(**model.to_evidence_dict()['source']['target_identity'])
            view = SurfaceView(identity, stamp, geometry.object_points_base, geometry.support_normal_base,
                geometry.support_offset_m, 12)
            surface = FusedTargetSurface(identity, geometry.object_points_base,
                np.zeros(len(geometry.object_points_base), dtype=np.int64), (stamp,))
            node._snapshot_multiview_surface = lambda: (surface, view)
            node._current_inferred_box = lambda: model
            node._regular_box_snapshot = dict(result=NS(audit=dict(model.qualification), ok=True,
                code='REGULAR_BOX_INFERENCE_QUALIFIED'), enclosure=enclosure)
            snapshot = NS(stamp_ns=stamp, target_identity=identity, quality=NS(
                depth_mad_m=table.get('depth_spatial_mad_m', 0.),
                depth_repeatability_m=table.get('depth_repeatability_m', 0.)))
            trace = Counter()
            interesting = {'_generate_tabletop_candidates', 'generate_tabletop_proposals', '_generate_inferred_proposals',
                'evaluate_inferred_box_contact', 'inferred_box_precision_reserve', '_obb_line_interval', 'materialize_tabletop_candidates'}
            def profile(frame, event, arg):
                if event == 'call' and frame.f_code.co_name in interesting:
                    trace[str(Path(frame.f_code.co_filename).resolve())+':'+frame.f_code.co_name] += 1
            sys.setprofile(profile)
            try:
                candidates, diagnostics = node._generate_tabletop_candidates(planning, snapshot)
            finally:
                sys.setprofile(None)
            assert not candidates and diagnostics['materialized_count'] == 0
            assert any(key.endswith(':_generate_inferred_proposals') for key in trace)
            assert not any(key.endswith(':materialize_tabletop_candidates') for key in trace)
            contact = []
            for axis in (0, 1):
                ev = evaluate_inferred_box_contact(model, model.center_base, model.axes_base[:,axis],
                    -model.support_normal_base, node.gripper_geometry,
                    support_normal_base=model.support_normal_base,
                    opening_fit_clearance_each_side_m=node.opening_fit_clearance_each_side_m)
                contact.append(dict(axis=axis, ok=ev.ok, code=ev.code, targeted_view=dict(ev.targeted_view)))
            assert all(abs(row['targeted_view']['precision_reserve_m']-.002)<1e-12 for row in contact)
            if name == 'GEN35':
                assert diagnostics['proposal_count'] == 0
                assert diagnostics['failure_code'] == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
                assert contact[1]['code'] == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
                narrow = contact[1]['targeted_view']
                assert abs(narrow['required_open_upper_m']-.04959335407014666)<1e-10
                assert abs(narrow['aperture_margin_m']-.00040664592985334425)<1e-10
            else:
                assert diagnostics['proposal_count'] == 1
                assert diagnostics['failure_code'] == 'GRIPPER_SWEEP_COLLISION'
                rejection, = diagnostics['proposal_reach_rejections']
                historical, = table['proposal_reach_rejections']
                assert rejection['failed_gate'] == 'finger_reach'
                for key in ('negative_reach_m', 'positive_reach_m', 'maximum_side_reach_m'):
                    assert abs(rejection[key]-historical[key])<1e-8
                assert rejection['negative_reach_m']+rejection['positive_reach_m']>.05
            assert point_hash() == audit['replay_geometry']['object_points_sha256']
            return dict(record_label=name, source_stamp_ns=stamp, target_identity=model.to_evidence_dict()['source']['target_identity'],
                archived_model_sha256=archived, replay_model_sha256=model.model_sha256,
                exact_historical_model_digest=exact, measured_point_count=len(geometry.object_points_base),
                measured_points_sha256=point_hash(), measured_points_unchanged=True,
                planning_enclosure=enclosure, candidate_count=len(candidates), actual_diagnostics=diagnostics,
                actual_contact_evaluations=contact, configured_reserve_m=node.gripper_geometry.jaw_clearance_each_side_m,
                physical_aperture_m=node.gripper_geometry.max_inner_gap_m,
                fit_clearance_each_side_m=node.opening_fit_clearance_each_side_m,
                tabletop_config=asdict(node.tabletop_geometry_config), call_trace=dict(trace),
                simulated_parameter_reads=parameter_reads, candidate_materialization_calls=0)

        orig33 = dict(data33['original_geometry'])
        orig33['points'] = orig33.pop('object_points_base')
        t35 = audit35['remote_diagnostics']['tabletop_geometry']
        en35, g35 = t35['regular_box_planning_enclosure'], audit35['replay_geometry']
        orig35 = dict(center_base=en35['original_center_base'], axes_base=en35['original_axes_base'],
            size_xyz_m=en35['original_size_xyz_m'], support_normal_base=g35['support_normal_base'],
            support_offset_m=g35['support_offset_m'], support_inlier_ratio=g35['support_inlier_ratio'],
            source_mode=g35['source_mode'], points=g35['object_points_base_m'])
        cases = [run_case('GEN33', audit33, data33['model'], orig33, False),
                 run_case('GEN35', audit35, routes35['actual_public_route']['model'], orig35, True)]
        after = {path: sha(path) for path in before}
        input_after = {path: sha(path) for path in inputs}
        assert before == after and inputs == input_after and not calls
        report.update(passed=True, cases=cases, manifests=manifests, source_before=before,
            source_after=after, source_unchanged=True, input_before=inputs, input_after=input_after,
            inputs_unchanged=True, external_boundary_attempts=calls,
            full_required_opening_includes_uncertainty_and_fit=True,
            strict_path_or_full_collision_checked=False, registration_or_source_admission_replayed=False,
            limitations=[
                'Current real tabletop candidate generation and inferred contact gates are called; learned candidates and the complete task FSM are outside this replay.',
                'GEN35 loads the immutable exact historical model reconstructed from retained source pixels in the separately sealed box-routes evidence. That earlier qualification is not repeated here.',
                'GEN33 uses the independently validated same-source canonical reconstruction; its digest differs from the historical model. It is not substituted into historical evidence.',
                'Frozen surface carriers retain every original measured point but do not reconstruct historical per-voxel lineage, registration or producer source admission.',
                'Source YAML supplies geometry settings; a complete historical remote parameter snapshot was not retained. Explicit aperture/fit and recorded rejection values are checked.',
                'GEN35 still has only 0.406646 mm total aperture margin against the existing configured 2 mm software reserve after removing duplicate uncertainty. This is not proof of physical impossibility.',
                'GEN33 still fails full union finger reach before materialization. No complete collision sweep, strict path, SDK, close, live robot or physical success is claimed.',
                'Socket/ROS/process/device boundary guards prevent unsimulated Python interfaces; zero attempts were observed.'])
    except BaseException as exc:
        sys.setprofile(None)
        report.update(error=repr(exc), traceback=traceback.format_exc(), source_before=before,
            input_before=inputs, external_boundary_attempts=calls)
    report['wall_sec'] = time.monotonic()-started
    (out/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'], result=str(out/'result.json'),
        sha256=sha(out/'result.json'), error=report.get('error')), ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
