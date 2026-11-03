"""Replay original box qualification and label a nonexecuting gate diagnostic.

Only the exact retained source pixels and archived fused geometry are read.
The optional in-memory diagnostic continues past the two already-rejected
RGB checks to expose the next original failure. It cannot qualify a model.
"""
import ast
import hashlib
import inspect
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import cuboid_rgb_interval as c
from alicia_flexible_grasp.vision import regular_box_inference as b
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.rgbd_snapshot import FrozenRgbdSample, SnapshotResult
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
NV = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G = E/'grasp_records/20261018T023244.888815Z_d5686c81c6144223b5adb93e60ca7cee'
I = NV/'gen43_actual_source_inspection_r1'
OUT = NV/'gen43_original_qualification_r2'
CASES = (
    (1, 'a4c568310c48ede8623c3d3347babb9b6a2d52de29af2292c3196b9354f9b8d2.json', 1792290768221260547),
    (18, 'b7d9a682a3f359f3963ecbf9e6fd95ecffa3768c8a55e81176f35da8b3451945.json', 1792290823025146722))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plain(value):
    return c._plain(value)


def inputs(request, name, stamp, evidence):
    path = G/'keyframes_audits'/name
    evidence[str(path)] = sha(path)
    report = json.loads(path.read_text())
    folder = I/('request'+str(request))/str(stamp)
    evidence[str(folder/'metadata.json')] = sha(folder/'metadata.json')
    meta = json.loads((folder/'metadata.json').read_text())
    arrays = {}
    for key in ('color_bgr', 'depth_raw', 'mask_uint8'):
        path = folder/(key+'.npy')
        evidence[str(path)] = sha(path)
        arrays[key] = np.load(path)
        assert b._array_hash(arrays[key]) == meta['files'][key]['decoded_array_sha256']
    expected = report['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
    assert all(expected['retained_pixel_sha256'][key+'_sha256'] == b._array_hash(arrays[array])
        for key, array in (('color', 'color_bgr'), ('depth', 'depth_raw'), ('mask', 'mask_uint8')))
    identity = TargetTrackIdentity(report['target_epoch'], expected['source_target_identity']['track_id'])
    sample_stamps = tuple(report['pipeline_funnel']['snapshot_evidence']['source_stamp_ns'])
    sample_masks = []
    for source_stamp in sample_stamps:
        path = I/('request'+str(request))/str(source_stamp)/'mask_uint8.npy'
        evidence[str(path)] = sha(path)
        sample_masks.append(b._array_hash((np.load(path) > 0).astype(np.uint8)))
    assert sample_masks[-1] == expected['final_sample_binary_mask_sha256']
    object_msg = meta['object_message']['value']
    box = tuple(object_msg[key] for key in ('bbox_x','bbox_y','bbox_width','bbox_height'))
    sample = FrozenRgbdSample(arrays['color_bgr'], arrays['depth_raw'], arrays['mask_uint8'],
        box, object_msg, report['snapshot_stamp_sec'], meta['files']['color_bgr']['frame_id'],
        None, stamp, identity.epoch, identity)
    # The box qualifier consumes original_sample plus admitted geometry. The
    # nonconsumed temporal-median depth and quality fields are not recreated
    # or claimed to be an execution-valid planning snapshot.
    snapshot = SnapshotResult(True, '', '', arrays['color_bgr'], arrays['depth_raw'],
        arrays['depth_raw'], arrays['mask_uint8'], box, object_msg,
        report['snapshot_stamp_sec'], sample.frame_id, None, 'instance_mask',
        stamp, identity.epoch, identity, sample_stamps, sample, tuple(sample_masks))
    assert b._retained_box_pixels(snapshot, identity, stamp)[0] is sample
    g = report['replay_geometry']
    geometry = GeometryEstimate(True, '', '', g['obb_center_base_m'], g['R_base_obb'],
        g['obb_size_xyz_m'], g['support_normal_base'], g['support_offset_m'],
        g['support_inlier_ratio'], g['object_points_base_m'], g['source_mode'])
    camera = SimpleNamespace(**meta['intrinsics'])
    T = np.array(report['snapshot_transform']['T_base_optical'])
    return report, expected, (snapshot, camera, camera.depth_scale, T, geometry, identity, stamp)


def diagnostic_clone():
    tree = ast.parse(inspect.getsource(c.infer_cuboid_rgb_interval))
    removed = []
    known = {
        'complete cuboid silhouette or corner arms exceed original raster gate',
        'complete cuboid silhouette is not covered by current mask'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Raise) and isinstance(node.exc, ast.Call) and node.exc.args:
            text = node.exc.args[0]
            if isinstance(text, ast.Constant) and text.value in known:
                removed.append(text.value)
                node.__class__ = ast.Expr
                node.__dict__ = dict(value=ast.Call(
                    func=ast.Attribute(value=ast.Name(id='diagnostic_skips', ctx=ast.Load()),
                                       attr='append', ctx=ast.Load()),
                    args=[ast.Constant(value=text.value)], keywords=[]),
                    lineno=node.lineno, col_offset=node.col_offset)
    assert set(removed) == known and len(removed) == 2
    namespace = dict(c.__dict__, diagnostic_skips=[])
    exec(compile(ast.fix_missing_locations(tree), '<GEN43 NONEXECUTING DIAGNOSTIC>', 'exec'), namespace)
    return namespace['infer_cuboid_rgb_interval'], namespace['diagnostic_skips']


def main():
    assert M.is_mount() and not OUT.exists() and OUT.parent.stat().st_dev == M.stat().st_dev
    policy = json.loads((E/'maintenance/storage_control.json').read_text())
    archive = json.loads((E/'grasp_records/archive_config.json').read_text())
    assert policy['archive_policy'] == 'local_only'
    assert archive['upload_enabled'] is False and archive['delete_enabled'] is False
    modules = [c, b, sys.modules[GeometryEstimate.__module__],
        sys.modules[SnapshotResult.__module__], sys.modules[TargetTrackIdentity.__module__]]
    evidence = {str(Path(m.__file__).resolve()): sha(m.__file__) for m in modules}
    evidence[str(Path(__file__).resolve())] = sha(__file__)
    rows = []
    original_validator = c.validate_cuboid_rgb_interval
    validator_tree = ast.parse(inspect.getsource(original_validator))
    matches = []
    class CoverageOnly(ast.NodeTransformer):
        def visit_Compare(self, node):
            if (isinstance(node.left, ast.Subscript) and isinstance(node.left.value, ast.Name)
                    and node.left.value.id == 'value'
                    and isinstance(node.left.slice, ast.Index)
                    and isinstance(node.left.slice.value, ast.Constant)
                    and node.left.slice.value.value == 'silhouette_mask_coverage'
                    and len(node.ops) == 1 and isinstance(node.ops[0], ast.Lt)
                    and isinstance(node.comparators[0], ast.Constant) and node.comparators[0].value == .88):
                matches.append(ast.dump(node))
                return ast.copy_location(ast.Constant(value=False), node)
            return self.generic_visit(node)
    validator_tree = CoverageOnly().visit(validator_tree)
    assert len(matches) == 1
    validator_namespace = dict(c.__dict__)
    exec(compile(ast.fix_missing_locations(validator_tree), '<GEN43 SAME-COVERAGE VALIDATOR DIAGNOSTIC>', 'exec'), validator_namespace)
    validator = validator_namespace['validate_cuboid_rgb_interval']
    original_infer = b.infer_cuboid_rgb_interval
    for request, filename, stamp in CASES:
        report, expected, args = inputs(request, filename, stamp, evidence)
        baseline = b.qualify_regular_box_snapshot(*args)
        assert baseline.ok is False and baseline.code == expected['code']
        assert baseline.audit.get('cuboid_qualification_reason') == expected['cuboid_qualification_reason']
        assert baseline.audit['retained_pixel_sha256'] == expected['retained_pixel_sha256']
        clone, skips = diagnostic_clone()
        b.infer_cuboid_rgb_interval = clone
        c.validate_cuboid_rgb_interval = b.validate_cuboid_rgb_interval = validator
        try:
            diagnostic = b.qualify_regular_box_snapshot(*args)
        finally:
            b.infer_cuboid_rgb_interval = original_infer
            c.validate_cuboid_rgb_interval = b.validate_cuboid_rgb_interval = original_validator
        rows.append(dict(request_id=request, source_stamp_ns=stamp,
            original_qualification=dict(ok=baseline.ok, code=baseline.code, audit=plain(dict(baseline.audit))),
            diagnostic_only_continuation=dict(ok_field_is_not_qualification=diagnostic.ok,
                code=diagnostic.code, skipped_previously_failed_gates=skips,
                same_coverage_predicate_in_JSON_validator_diagnostically_ignored=True,
                all_other_validator_checks_unchanged=True,
                audit=plain(dict(diagnostic.audit))),
            complete_original_source_binding_rechecked=True,
            temporal_median_depth_recreated=False, execution_snapshot_recreated=False,
            qualified_model_or_motion_authorized=False))
        print(json.dumps(dict(request_id=request, baseline_code=baseline.code,
            diagnostic_code=diagnostic.code, skips=skips,
            next_reason=diagnostic.audit.get('cuboid_qualification_reason', diagnostic.audit.get('reason')))), flush=True)
    assert all(sha(path) == digest for path, digest in evidence.items())
    OUT.mkdir()
    result = dict(schema=1, created_ns=time.time_ns(),
        kind='original_qualification_and_explicit_nonexecuting_gate_diagnostic',
        source_and_inputs_sha256=evidence, requests=rows,
        production_sources_changed=False, nominal_fitting_changed=False,
        original_gate_validation_passed=False, qualified_model_created=False,
        ROS_called=False, hardware_contacted=False, motion_commands_sent=False, temperature_queried=False)
    path = OUT/'result.json'
    with path.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    print(json.dumps(dict(result=str(path), sha256=sha(path))))


if __name__ == '__main__':
    main()
