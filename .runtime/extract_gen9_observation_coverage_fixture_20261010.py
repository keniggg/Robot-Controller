"""Extract exact GEN9 audited candidate rows; never call ROS or a planner."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import numpy as np
import rosbag
from tf.transformations import quaternion_matrix

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_restart_20261010T002347Z'
REC = EXT / 'grasp_records/20261010T042204.923624Z_7543ac3513a6431496aae778c0561e24'
AUDIT = REC / 'keyframes_audits/25f96210ea2d2d043a0a1624a492129915e81426ceaf0e25dad9e6eb9fc1428b.json'
STAGE = WORK / '.runtime/gen9_silhouette_coverage_staging_20261010'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
data = json.loads(AUDIT.read_text())
assert sha(AUDIT) == AUDIT.stem
manifest = json.loads((REC / 'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
failure = data['observation_failure_diagnostics']
mount, tf_source = None, None
with rosbag.Bag(str(REC / 'keyframes.bag'), 'r') as bag:
    for _, message, received in bag.read_messages(topics=['/tf_static']):
        for transform in message.transforms:
            if (transform.header.frame_id, transform.child_frame_id) != ('tool0', 'camera_link'):
                continue
            translation = [getattr(transform.transform.translation, key) for key in 'xyz']
            rotation = np.asarray([getattr(transform.transform.rotation, key) for key in 'xyzw'])
            rotation = rotation / np.linalg.norm(rotation)
            matrix = quaternion_matrix(rotation)
            matrix[:3, 3] = translation
            if mount is not None:
                assert np.array_equal(mount, matrix)
            else:
                mount = matrix
                tf_source = dict(bag=str(REC / 'keyframes.bag'), topic='/tf_static',
                    receipt_ns=received.to_nsec(), stamp_ns=transform.header.stamp.to_nsec(),
                    parent='tool0', child='camera_link', translation_xyz=translation,
                    quaternion_xyzw=rotation.tolist(), exact_original_source_hash_verified=True)
assert mount is not None
mount_sha = hashlib.sha256(mount.astype('<f8').tobytes()).hexdigest()
assert mount_sha == failure['source']['captured_audit']['tool_camera_sha256']
candidates = failure['observation_candidates']
search = failure['observation_orientation_search']
assert len(candidates) == 47 and search['checked_variant_count'] == 4
assert all(c['observation_envelope']['ok'] for c in candidates)
assert all(c['observation_view']['check_scheduling_metrics']['endpoint_ik_hint']['error_code'] == -31
           for c in candidates)
ordered = sorted(candidates, key=lambda c: c['observation_view']['check_scheduling_index'])
assert [c['observation_view']['check_scheduling_index'] for c in ordered] == list(range(47))
for c, r in zip(ordered, search['variant_results']):
    assert all(c[k] == r[k] for k in (
        'camera_roll_offset_deg', 'camera_tilt_offset_deg', 'camera_target_distance_m'))
fixture = dict(version=1, source_record=str(REC), source_audit=str(AUDIT), source_audit_sha256=sha(AUDIT),
    source_transforms=dict(tool_from_camera=mount.tolist(), tool_from_camera_sha256=mount_sha,
        provenance=tf_source),
    request={k: data.get(k) for k in ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns',
        'grasp_mode_selection', 'planning_origin', 'observation_motion_performed')},
    source=failure['source'], scheduling=failure['unknown_observation_scheduling'],
    observation_candidates=candidates, observation_orientation_search=search,
    observation_endpoint_rejections=failure['observation_endpoint_rejections'],
    limitations=['Exact JSON subtrees from the sealed gate audit; no candidate transform or measured field is generated.',
        'Only four candidates have original strict path results. All other poses are untested, not proven unreachable.',
        'All 47 endpoint IK hints returned -31 at 20 ms; this is unknown, not proof of unreachable pose.',
        'No current scene is substituted for the recorded scene; no ROS service, motion or scene mutation is performed.'])
fixture_path = STAGE / 'src/alicia_grasp_modes/tests/fixtures/gen9_silhouette_coverage.json'
fixture_path.parent.mkdir(parents=True, exist_ok=True)
fixture_path.write_text(json.dumps(fixture, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
distances = []
for distance in sorted({c['camera_target_distance_m'] for c in candidates}):
    rows = [c for c in ordered if c['camera_target_distance_m'] == distance]
    first = rows[0]
    metrics = first['observation_view']['check_scheduling_metrics']
    distances.append(dict(camera_target_distance_m=distance, candidate_count=len(rows),
        checked_count=sum(1 for r in search['variant_results'] if r.get('checked') and r['camera_target_distance_m'] == distance),
        earliest_old_check_index=first['observation_view']['check_scheduling_index'],
        earliest_candidate_index=first['candidate_index'], earliest_roll_deg=first['camera_roll_offset_deg'],
        earliest_tool_position_m=[row[3] for row in first['base_from_tool'][:3]],
        earliest_original_metrics=metrics))
report = dict(source_audit=str(AUDIT), source_audit_sha256=sha(AUDIT),
    fixture=dict(path=str(fixture_path), sha256=sha(fixture_path)),
    request=fixture['request'], checked=search['variant_results'][:4], distance_coverage=distances,
    endpoint_rejection_counts=dict(Counter(r['failure_code'] for r in failure['observation_endpoint_rejections'])),
    untested_reasons=dict(Counter(r.get('reason') for r in search['variant_results'] if not r.get('checked'))),
    scheduling=failure['unknown_observation_scheduling'],
    conclusions=[
        'Four actual strict planning calls returned MOVEIT_UNREACHABLE / OBSERVATION_POSE_PLAN_FAILED; no transport-unavailable evidence.',
        'The diagnostic branch uses legacy tilt budget. All 47 candidates have tilt 45 degrees, so all consume that shared budget. Four checked is below the six-check ceiling; with no disabled timeout reported the three-second accumulated strict-work budget is the consistent stop condition. Exact used_sec is not persisted in this failure audit.',
        'SO3 rotation is the first fallback sort key; support clearance precedes translation for equal rotations. This prioritizes roll zero at long distance, then +/-15 degrees at the longest distance. Nearer camera positions require larger roll to pass original endpoint geometry, and receive no strict checks before budget exhaustion.',
        'Five distinct distances remain after original endpoint gates. The earliest candidates at 285, 269.822 and 196.188 mm have positive original support clearance and predicted side evidence, but no recorded strict path result.',
        'The minimum scoped fix is fair distance/direction scheduling only for diagnostic silhouette work with no positive endpoint hint until the first fully qualified incumbent. Keep all original candidates, strict budget, source reserve, endpoint/support/collision/following/path checks, and normal post-incumbent ranking.',
    ],
    limitations=fixture['limitations'], production_changed=False, physical_actions=False)
out = NEW / 'gen9_handoff_analysis/observation_coverage'
out.mkdir(parents=True, exist_ok=True)
dest = out / 'recorded_candidate_assessment.json'
dest.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
print(json.dumps(dict(fixture=str(fixture_path), fixture_sha256=sha(fixture_path),
    report=str(dest), report_sha256=sha(dest)), ensure_ascii=False))
