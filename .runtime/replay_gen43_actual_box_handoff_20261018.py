"""Replay exact G42 raw source through unmodified qualification and aperture math.

Choose an isolated source workspace. No rejected gate is skipped and no
diagnostic model is substituted. This grants no execution authority.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import yaml

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
NV = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G = E/'grasp_records/20261018T023244.888815Z_d5686c81c6144223b5adb93e60ca7cee'
I = NV/'gen43_actual_source_inspection_r1'
CASES = ((1, 'a4c568310c48ede8623c3d3347babb9b6a2d52de29af2292c3196b9354f9b8d2.json', 1792290768221260547),
         (18, 'b7d9a682a3f359f3963ecbf9e6fd95ecffa3768c8a55e81176f35da8b3451945.json', 1792290823025146722))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    work, out = args.workspace.resolve(strict=True), args.output.resolve()
    assert work in (W/'.runtime/gen42_source_before_gen43', W/'.runtime/gen43_mask_candidate')
    assert M.is_mount() and not out.exists()
    out.relative_to(NV.resolve(strict=True))
    assert out.parent.stat().st_dev == M.stat().st_dev
    archive = json.loads((E/'grasp_records/archive_config.json').read_text())
    policy = json.loads((E/'maintenance/storage_control.json').read_text())
    assert archive['upload_enabled'] is False and archive['delete_enabled'] is False
    assert policy['archive_policy'] == 'local_only'
    sys.path.insert(0, str(work/'src/alicia_flexible_grasp_supervisor/src'))
    # Load the original pure transformation math without importing tf's ROS
    # listener/client package. No transform, gripper or proposal is simulated.
    transform_path = Path('/opt/ros/noetic/lib/python3/dist-packages/tf/transformations.py')
    assert transform_path.is_file() and 'tf' not in sys.modules and 'rospy' not in sys.modules
    namespace = ModuleType('tf'); namespace.__path__ = [str(transform_path.parent)]
    sys.modules['tf'] = namespace
    spec = importlib.util.spec_from_file_location('tf.transformations', transform_path)
    transform = importlib.util.module_from_spec(spec)
    sys.modules['tf.transformations'] = transform
    spec.loader.exec_module(transform)
    from alicia_flexible_grasp.vision import regular_box_inference as box
    from alicia_flexible_grasp.vision import cuboid_rgb_interval as cuboid
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample, SnapshotResult
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.grasp import gripper_geometry as grip
    from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tabletop
    evidence = {str(Path(__file__).resolve()): sha(__file__), str(transform_path): sha(transform_path)}
    release_path = NV/'gen42_release_manifest.json'
    assert sha(release_path) == '4ce3b5142b18e9968deff461d39a202f0fe6f91572394744abad67d29c4810dc'
    release = json.loads(release_path.read_text())
    evidence[str(release_path)] = sha(release_path)
    cuboid_relative = str(Path(cuboid.__file__).resolve().relative_to(work))
    expected_cuboid = ('50e00a5b751ba0470f98c4215922bb6f05471784a364181c297ad5f617751d7f'
        if work.name == 'gen43_mask_candidate' else release['source_sha256'][cuboid_relative])
    assert sha(cuboid.__file__) == expected_cuboid
    modules = [box, cuboid, grip, tabletop, sys.modules[GeometryEstimate.__module__],
               sys.modules[SnapshotResult.__module__], sys.modules[TargetTrackIdentity.__module__]]
    for module in modules:
        path = Path(module.__file__).resolve(strict=True)
        path.relative_to(work/'src')
        evidence[str(path)] = sha(path)
    config_relative = 'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
    config_path = work/config_relative
    assert sha(config_path) == release['source_sha256'][config_relative]
    evidence[str(config_path)] = sha(config_path)
    config_values = yaml.safe_load(config_path.read_text())
    remote_cfg = config_values['grasp_6d']['remote']
    geometry_cfg = remote_cfg['gripper_geometry']
    tabletop_cfg = remote_cfg['tabletop_geometry_candidates']
    opening_cfg = config_values['gripper']
    fit_clearance = geometry_cfg['opening_fit_clearance_each_side_m']
    gripper = grip.GripperGeometry(geometry_cfg['max_inner_gap_m'],
        geometry_cfg['width_safety_margin_per_side_m'], geometry_cfg['finger_box_xyz_m'],
        geometry_cfg['palm_box_xyz_m'], tabletop_cfg['min_finger_support_clearance_m'])
    assert gripper.max_inner_gap_m == opening_cfg['open_position_m'] == .05
    assert fit_clearance == .0005
    config = tabletop.TabletopGeometryConfig(
        max_inner_gap_m=gripper.max_inner_gap_m,
        angle_step_deg=tabletop_cfg['angle_step_deg'], angle_dedup_deg=tabletop_cfg['angle_dedup_deg'],
        jaw_clearance_each_side_m=tabletop_cfg['jaw_clearance_each_side_m'],
        opening_fit_clearance_each_side_m=fit_clearance,
        width_projection_trim_fraction=tabletop_cfg['width_projection_trim_fraction'],
        min_contact_band_points=tabletop_cfg['min_contact_band_points'],
        contact_band_fraction=tabletop_cfg['contact_band_fraction'],
        max_candidates=tabletop_cfg['max_candidates'],
        approach_tilt_degrees=tuple(tabletop_cfg['approach_tilt_degrees']))
    execution_path = G/'perception_execution_configuration.json'
    evidence[str(execution_path)] = sha(execution_path)
    execution = json.loads(execution_path.read_text())['parameters']['/grasp']
    assert execution['unknown_require_measured_bilateral_contact'] is False
    startup_basis = dict(sealed_configuration=str(config_path),
        sealed_configuration_sha256=sha(config_path),
        original_perception_execution_configuration=str(execution_path),
        full_live_remote_parameter_dump_available=False,
        configured_open_position_m=opening_cfg['open_position_m'],
        configured_open_skip_tolerance_m=opening_cfg['open_skip_tolerance_m'],
        permitted_open_position_interval_m=[opening_cfg['open_position_m']-opening_cfg['open_skip_tolerance_m'],opening_cfg['open_position_m']],
        actual_open_encoder_value_basis='G42 request1 endpoint_ik_search_probe joint_solution reports 0.0498m',
        archived_encoder_open_position_m=.0498,
        opening_fit_clearance_each_side_m=fit_clearance,
        finger_box_xyz_m=geometry_cfg['finger_box_xyz_m'], palm_box_xyz_m=geometry_cfg['palm_box_xyz_m'],
        physical_collision_jaw_clearance_each_side_m=geometry_cfg['width_safety_margin_per_side_m'],
        min_support_clearance_m=tabletop_cfg['min_finger_support_clearance_m'],
        contact_overlap_min_m=remote_cfg['adaptive_stage_generation']['contact_overlap_min_m'],
        archived_handeye_translation_max_error_m=execution['handeye_translation_max_error_m'],
        tabletop_config=tabletop_cfg,
        limitation='Sealed startup YAML plus archived grasp configuration and actual audit stage budgets; no claim of unrecorded live parameter equality or calibration validity.')
    results = []
    for request, filename, stamp in CASES:
        audit_path = G/'keyframes_audits'/filename
        evidence[str(audit_path)] = sha(audit_path)
        report = json.loads(audit_path.read_text())
        folder = I/('request'+str(request))/str(stamp)
        meta_path = folder/'metadata.json'; evidence[str(meta_path)] = sha(meta_path)
        meta = json.loads(meta_path.read_text())
        arrays = {}
        for name in ('color_bgr','depth_raw','mask_uint8'):
            path = folder/(name+'.npy'); evidence[str(path)] = sha(path)
            arrays[name] = np.load(path)
            assert box._array_hash(arrays[name]) == meta['files'][name]['decoded_array_sha256']
        archived = report['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
        identity = TargetTrackIdentity(report['target_epoch'], archived['source_target_identity']['track_id'])
        stamps = tuple(report['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'])
        masks = []
        for source in stamps:
            path = I/('request'+str(request))/str(source)/'mask_uint8.npy'
            evidence[str(path)] = sha(path)
            masks.append(box._array_hash((np.load(path)>0).astype(np.uint8)))
        assert masks[-1] == archived['final_sample_binary_mask_sha256']
        object_msg = meta['object_message']['value']
        bbox = tuple(object_msg[key] for key in ('bbox_x','bbox_y','bbox_width','bbox_height'))
        sample = FrozenRgbdSample(arrays['color_bgr'], arrays['depth_raw'], arrays['mask_uint8'],
            bbox, object_msg, report['snapshot_stamp_sec'], meta['files']['color_bgr']['frame_id'],
            None, stamp, identity.epoch, identity)
        snapshot = SnapshotResult(True, '', '', arrays['color_bgr'], arrays['depth_raw'],
            arrays['depth_raw'], arrays['mask_uint8'], bbox, object_msg, report['snapshot_stamp_sec'],
            sample.frame_id, None, 'instance_mask', stamp, identity.epoch, identity,
            stamps, sample, tuple(masks))
        assert box._retained_box_pixels(snapshot, identity, stamp)[0] is sample
        g = report['replay_geometry']
        geometry = GeometryEstimate(True, '', '', g['obb_center_base_m'], g['R_base_obb'],
            g['obb_size_xyz_m'], g['support_normal_base'], g['support_offset_m'],
            g['support_inlier_ratio'], g['object_points_base_m'], g['source_mode'])
        source_digest = box.regular_box_geometry_sha256(geometry)
        camera = SimpleNamespace(**meta['intrinsics'])
        T = np.array(report['snapshot_transform']['T_base_optical'])
        result = box.qualify_regular_box_snapshot(snapshot, camera, camera.depth_scale,
            T, geometry, identity, stamp)
        row = dict(request_id=request, source_stamp_ns=stamp, source_identity=dict(epoch=identity.epoch,
            track_id=identity.track_id), original_source_binding_rechecked=True,
            source_geometry_sha256=source_digest, qualification_ok=result.ok,
            qualification_code=result.code, qualification_audit=cuboid._plain(dict(result.audit)),
            rejected_gate_skipped=False, inferred_model_substituted=False,
            temporal_median_depth_recreated=False, execution_snapshot_recreated=False,
            grasp_plan_generated=False, execution_authorized=False,
            archived_adaptive_stage_profiles=report['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles'],
            actual_contact_overlap_requirement_m=report['remote_diagnostics']['tabletop_geometry']['required_contact_patch_overlap_m'],
            source_calibration_association=meta['intrinsics_evidence'])
        if result.ok:
            model = result.model
            assert model.source['geometry_sha256'] == source_digest
            assert model.source['stamp_ns'] == stamp
            planning, enclosure = box.enclose_inferred_box_geometry(geometry, model)
            assert np.array_equal(planning.object_points_base, geometry.object_points_base)
            proposals = tabletop.generate_tabletop_proposals(
                geometry.object_points_base, geometry.center_base, geometry.axes_base, geometry.size_xyz_m,
                -geometry.support_offset_m*geometry.support_normal_base, geometry.support_normal_base,
                config=config, inferred_box=model, finger_geometry=gripper)
            contacts = []
            for proposal in proposals.proposals:
                check = grip.evaluate_inferred_box_contact(model, proposal.contact_center_base,
                    proposal.jaw_axis_base, proposal.insertion_axis_base, gripper,
                    support_normal_base=geometry.support_normal_base,
                    opening_fit_clearance_each_side_m=fit_clearance)
                contacts.append(dict(source_index=proposal.source_index, angle_deg=proposal.angle_deg,
                    ok=check.ok, code=check.code, nominal_width_m=check.inferred_width_m,
                    full_width_uncertainty_m=check.width_uncertainty_m,
                    required_open_width_m=proposal.required_open_width_m,
                    aperture_margin_m=proposal.aperture_margin_m,
                    aperture_margin_to_archived_encoder_opening_m=.0498-proposal.required_open_width_m,
                    aperture_margin_to_minimum_permitted_opening_m=(opening_cfg['open_position_m']-opening_cfg['open_skip_tolerance_m'])-proposal.required_open_width_m,
                    contact_height_bounds_m=list(check.contact_height_bounds_m),
                    evidence_kind='regular_box_model_inference_not_measured_bilateral'))
            row.update(model_sha256=model.model_sha256, model_evidence=model.to_evidence_dict(),
                planning_enclosure=enclosure, proposal_code=proposals.failure_code,
                proposal_reason=proposals.failure_reason, proposal_count=len(proposals.proposals),
                contact_projections=contacts, physical_max_gap_m=gripper.max_inner_gap_m,
                complete_measured_and_family_enclosure_checked=True)
        results.append(row)
        print(json.dumps(dict(request=request, qualification_ok=result.ok, code=result.code,
            proposal_count=row.get('proposal_count'), contact_projections=row.get('contact_projections'))), flush=True)
    assert 'rospy' not in sys.modules
    assert all(sha(path) == digest for path, digest in evidence.items())
    out.mkdir()
    record = dict(schema=1, created_ns=time.time_ns(), workspace=str(work),
        kind='actual_source_original_qualification_aperture_enclosure_replay', requests=results,
        source_and_inputs_sha256=evidence, original_qualification_functions_unchanged=True,
        startup_configuration_and_budget_basis=startup_basis,
        diagnostic_gate_bypass_used=False, physical_grasp_verified=False,
        ROS_called=False, hardware_contacted=False, motion_commands_sent=False, temperature_queried=False)
    path = out/'result.json'
    with path.open('x') as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    print(json.dumps(dict(result=str(path), sha256=sha(path))))


if __name__ == '__main__':
    main()
