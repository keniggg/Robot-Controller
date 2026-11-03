"""Read-only frozen GEN53 tabletop normalization/scoring; no service authority."""
import ast
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import signal
import struct
import sys
import time
from copy import deepcopy
from dataclasses import asdict
from types import SimpleNamespace

import extract_gen50_retry_r30_rgb_20261018 as boundary

W = Path(__file__).resolve().parents[1]
C = W / '.runtime/gen54_release_candidate_r1'
OUT = boundary.N / 'verification/gen54_geometry_pipeline_r1' / (sys.argv[1] if len(sys.argv) > 1 else 'normalized_scoring_r3')


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main():
    assert boundary.E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
                      OMP_NUM_THREADS='1', TMPDIR=str(OUT), ROS_HOME=str(OUT),
                      ROS_LOG_DIR=str(OUT), ROS_MASTER_URI='http://127.0.0.1:1',
                      ROS_PACKAGE_PATH=str(C / 'src') + ':/opt/ros/noetic/share')
    sys.dont_write_bytecode = True
    n = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(n) + (n.machine,)))
    platform.architecture = lambda *a, **kw: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
    boundary.OUT = OUT
    sys.addaudithook(boundary.audit)
    signal.signal(signal.SIGALRM, lambda *a: (_ for _ in ()).throw(TimeoutError('90 second offline replay budget')))
    signal.alarm(90)
    sys.path[:0] = list(map(str, [C/'src/alicia_flexible_grasp_supervisor/src',
        C/'src/alicia_grasp_modes/src', C/'devel/lib/python3/dist-packages',
        Path('/opt/ros/noetic/lib/python3/dist-packages')]))
    import numpy as np
    import yaml
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plain
    from gen54_pipeline_materializer import create_frozen_node, generate_local

    receipt = json.loads((OUT.parent/'exact_inputs/result.json').read_text())
    audit_path = Path(receipt['audit']['path'])
    assert boundary.sha(audit_path) == receipt['audit']['sha256']
    audit = json.loads(audit_path.read_text())
    g = audit['replay_geometry']
    geometry = GeometryEstimate(True, '', '', np.array(g['obb_center_base_m']),
        np.array(g['R_base_obb']), np.array(g['obb_size_xyz_m']),
        np.array(g['support_normal_base']), g['support_offset_m'],
        g['support_inlier_ratio'], np.array(g['object_points_base_m']), g['source_mode'])
    ref = audit['multiview_surface']['fused_scene_geometry']['surface_replay']['references']['snapshot']
    zipped = (Path(ref['store_root'])/ref['relative_path']).read_bytes()
    raw = gzip.decompress(zipped)
    assert hashlib.sha256(zipped).hexdigest() == ref['gzip_sha256']
    assert hashlib.sha256(raw).hexdigest() == ref['object_sha256']
    snapshot_surface = json.loads(raw)
    identity = TargetTrackIdentity(**snapshot_surface['identity'])
    surface = FusedTargetSurface(identity, np.array(snapshot_surface['points_base']),
        np.array(snapshot_surface['view_indices']), tuple(snapshot_surface['view_stamps_ns']),
        tuple(tuple(v) for v in snapshot_surface['view_sources']))
    reference = SurfaceView(identity=identity, **{k:v for k,v in snapshot_surface['reference'].items() if k != 'identity'})
    frozen, env = create_frozen_node(C, audit, geometry, surface, reference)
    began = time.monotonic()
    materialized, generation = generate_local(frozen, env, geometry)
    root = C/'src/alicia_flexible_grasp_supervisor'
    remote = load_module('_alicia_original_remote_grasp6d', root/'scripts/remote_grasp6d_node.py')
    # Read-only logging has no ROS node/master initialization.
    logs = []
    for name in ('logwarn', 'loginfo', 'logerr', 'logwarn_throttle', 'loginfo_throttle'):
        setattr(remote.rospy, name, lambda *a, **kw: logs.append([str(x) for x in a]))
    mode = load_module('_gen54_frozen_mode', C/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py')
    node = mode.ModeAwareRemoteGrasp6DNode.__new__(mode.ModeAwareRemoteGrasp6DNode)
    node.__dict__.update(frozen.__dict__)
    config_path = root/'config/grasp_params.yaml'
    config = yaml.safe_load(config_path.read_text())
    cfg = config['grasp_6d']['remote']
    handeye_path = root/'config/handeye.yaml'
    handeye = yaml.safe_load(handeye_path.read_text())['handeye']
    nonimage = json.loads(Path(receipt['nonimage_context']['path']).read_text())
    camera_row = next(r for r in nonimage if r['topic'] == '/grasp_record/camera_runtime_metadata')
    camera_cfg = camera_row['parsed_json']['parameters']
    def frozen_parameter(name, default=None):
        if name == '/camera':
            return deepcopy(camera_cfg)
        raise AssertionError('Unexpected ROS parameter request: '+name)
    remote.rospy.get_param = frozen_parameter
    node.grasp_config = config['grasp']
    node.handeye_translation_xyz = handeye['translation_xyz']
    node.handeye_rotation_xyzw = handeye['rotation_xyzw']
    node.gripper_physical_open_width_m = config['gripper']['open_position_m']
    node.soft_score_weights = remote.SoftScoreWeights(**cfg.get('soft_score_weights', {}))
    node.target_absolute_sanity_distance_m = cfg['target_absolute_sanity_distance_m']
    node.target_instance_epoch = identity.epoch
    for name in ('candidate_min_downward_approach_cos', 'candidate_max_final_approach_lateral_m',
                 'camera_visibility_gate_enabled', 'camera_visibility_diagnostic_enabled',
                 'camera_visibility_require_approach', 'camera_visibility_margin_px',
                 'camera_visibility_bbox_padding_px', 'camera_visibility_min_depth_m',
                 'camera_visibility_max_depth_m'):
        setattr(node, name, cfg[name])
    recorded_phase = audit['observe_return'].get('phase', '')
    # The production audit intentionally renders observing_contact as
    # observation_contact (observe_return_remote.py's audit serializer).
    node._observe_return_phase = {'observation_contact':'observing_contact'}.get(recorded_phase, recorded_phase)
    node._observe_return_diagnostic_observation_only = False
    node.__dict__.pop('_direct_strategy_selected', None)
    assert node._direct_strategy_selected() is True
    # Exact archived metadata is emitted only for a qualified measurement.
    # The generation branch consumes just this state; no receipt authority is
    # created or used for any downstream action in this offline process.
    assert audit['observe_return']['measurement_receipt_sha256']
    node._measurement_remote_request = {'state':'qualified'}
    full_mode_materialized, full_mode_generation = node._generate_tabletop_candidates(geometry)
    assert len(full_mode_materialized) == len(materialized)
    assert all(a.source_index == b.source_index and a.variant_index == b.variant_index
               and np.array_equal(a.T_base_tool0, b.T_base_tool0)
               for a,b in zip(materialized, full_mode_materialized))
    materialized, generation = full_mode_materialized, full_mode_generation
    node._active_graspnet_input_audit = {}
    stamp_ns = audit['snapshot_stamp_ns']
    stamp = remote.rospy.Time(stamp_ns//1000000000, stamp_ns%1000000000)
    prepared = SimpleNamespace(geometry=geometry, stamp=stamp,
        snapshot=SimpleNamespace(target_identity=identity, stamp_ns=stamp_ns,
            object_msg=SimpleNamespace(label='unknown_small', detected=True),
            quality=SimpleNamespace(depth_repeatability_m=generation['depth_repeatability_m'],
                depth_mad_m=generation['depth_spatial_mad_m'])),
        near_field=True, candidates=(), tabletop_candidates=materialized,
        model_choice='unknown_tabletop', remote_diagnostics={'tabletop_geometry': generation},
        remote_failure_code='', request_invalidation_generation=audit['generation'],
        ticket=SimpleNamespace(request_id=audit['request_id'], generation=audit['generation'],
            target_epoch=identity.epoch, snapshot_stamp_sec=audit['snapshot_stamp_sec']),
        pose_estimator=SimpleNamespace(**{k:np.asarray(v) if k.startswith('T_') else v
            for k,v in audit['snapshot_transform'].items()}))
    observations, local_funnel = node._evaluate_local_candidates(prepared)
    stable = node._single_request_stable_candidates(observations)
    scored = node._recheck_and_score_stable(prepared, stable)
    dedupe = {}
    queue = node._dedupe_exact_contact_sequences_for_moveit(scored, prepared, acceptance_diagnostics=dedupe)
    direction = remote.contact_direction_priorities(queue, node._direct_near_field_moveit_rank_key, {},
        observation_phase=node._observe_return_phase)
    ranked = sorted(queue, key=lambda c:(1, direction.get((c.track_id,c.variant_index),0),
        *node._direct_near_field_moveit_rank_key(c)))
    def normalized_row(o):
        n = o.payload
        return dict(source_index=n.source_index, materialized_variant_index=n.variant_index,
            required_open_width_m=n.required_open_width_m, common_physical_cost=n.common_physical_cost,
            gate=asdict(n.geometry_gate), audit=dict(n.audit), T_base_tool0=n.T_base_tool0)
    def scored_row(c):
        runtime = node._stable_variant_runtime[(c.track_id, c.variant_index)]
        return dict(track_id=c.track_id, recheck_variant_index=c.variant_index,
            source_index=c.payload.source_index, materialized_variant_index=c.payload.variant_index,
            rank_key=node._direct_near_field_moveit_rank_key(c),
            direction_priority=direction.get((c.track_id,c.variant_index),0),
            required_open_width_m=c.stable_candidate.required_open_width_m,
            mandatory_safety=asdict(remote.mandatory_safety_gate(c.latest_safety)),
            pre_moveit_score=c.pre_moveit_score, contact_height_placement=runtime['contact_height_placement'],
            soft_evidence=runtime['soft_evidence'], sequence=node._execution_sequence_audit(runtime['sequence'], True))
    result = dict(offline_only=True, execution_authorized=False, physical_grasp_success=False,
        source_root=str(C), source_audit_sha256=boundary.sha(audit_path), source_surface_reference=ref,
        full_production_normalization_and_stable_scoring=True,
        full_mode_generation_same_32_matrices=True,
        scope='Frozen automatic tabletop batch; original normalization, mandatory safety, current support/height recheck, mode-aware exact family deduplication, and fallback preflight ranking. No ROS/TF/service/MoveIt/held/lift execution.',
        limitations=['GraspNet branch excluded; this diagnostic addresses the new tabletop candidates only.',
            'Endpoint IK priorities require the live compute_ik service; no hint results fabricated. Queue order assumes all hints unavailable.',
            'Frozen source YAML used with archived camera metadata and dynamic materialization inputs, not a full historical ROS parameter reconstruction.',
            'No strict service reply or final registered full sequence success is claimed.'],
        materialized_count=len(materialized), normalized_count=len(observations), stable_count=len(stable),
        scored_count=len(scored), deduplicated_count=len(queue),
        local_funnel=local_funnel, recheck_audit=node._stable_recheck_audit_for(prepared),
        deduplication=dedupe, observations=[normalized_row(o) for o in observations],
        ranked_queue=[scored_row(c) for c in ranked], captured_logs=logs,
        configuration=dict(soft_score_weights=asdict(node.soft_score_weights),
            camera_visibility_gate_enabled=node.camera_visibility_gate_enabled,
            camera_visibility_diagnostic_enabled=node.camera_visibility_diagnostic_enabled,
            camera_intrinsics={k:camera_cfg[k] for k in ('width','height','fx','fy','cx','cy','depth_scale')},
            handeye=handeye, observation_phase=node._observe_return_phase,
            archived_audit_phase=recorded_phase,
            archived_qualified_receipt_sha256=audit['observe_return']['measurement_receipt_sha256']),
        elapsed_sec=time.monotonic()-began, boundary_attempts=boundary.BOUNDARY,
        source_hashes={str(p):boundary.sha(p) for p in [config_path, handeye_path,
            root/'scripts/remote_grasp6d_node.py', C/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py']},
        helper_sha256=boundary.sha(__file__), ROS_initialized=False, hardware_contacted=False)
    assert not boundary.BOUNDARY
    (OUT/'result.json').write_text(json.dumps(_plain(result),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('materialized_count','normalized_count','stable_count','scored_count','deduplicated_count','elapsed_sec')}))
    print(json.dumps([dict(source_index=r['source_index'],materialized_variant_index=r['materialized_variant_index'],
        recheck_variant_index=r['recheck_variant_index'],rank_key=r['rank_key']) for r in _plain(result)['ranked_queue']],ensure_ascii=False))
    signal.alarm(0)


if __name__ == '__main__':
    main()
