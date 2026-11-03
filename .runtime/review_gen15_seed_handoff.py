"""Independent closed-record/source review. No ROS services or publishers."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import rosbag
import rospy

WORK = Path(__file__).resolve().parents[1]
STAGE = WORK / '.runtime/gen15_seed_handoff_20261011'
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
artifact = lambda p: dict(path=str(p), sha256=sha(p))
delivery_path = STAGE / 'delivery.json'
delivery = json.loads(delivery_path.read_text())
assert delivery['ready'] and delivery['source_frozen']
assert sha(delivery_path) == '2179cca985b7f14832a16b3f95e9eab2a801722b387619d19c9c45bf7f332a7e'
sources = dict(delivery['files'], **delivery['test_files'])
assert sources == delivery['source_sha256']
assert all(sha(STAGE / p) == digest for p, digest in sources.items())
prod = next(iter(delivery['files']))
old = WORK / '.runtime/gen14_fk_early_qualified_20261011' / prod
assert sha(old) == delivery['base_sha256'][prod]

def without_changed_method(path):
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ModeAwareRemoteGrasp6DNode')
    cls.body = [n for n in cls.body if not isinstance(n, ast.FunctionDef)
                or n.name != '_joint_space_observation_candidates']
    return ast.dump(tree, include_attributes=False)

assert without_changed_method(old) == without_changed_method(STAGE / prod)
fixture_path = STAGE / 'src/alicia_grasp_modes/tests/fixtures/gen15_recorded_seed_handoff.json'
fixture = json.loads(fixture_path.read_text())
audit_path = Path(fixture['audit_path'])
assert sha(audit_path) == fixture['audit_sha256']
audit = json.loads(audit_path.read_text())['observation_failure_diagnostics']
assert fixture['source_audit'] == audit['source']['captured_audit']
assert fixture['observed_generator_audit'] == audit['joint_space_fk_search']['audit']
assert fixture['source_binding'] == audit['joint_space_fk_search']['audit']['binding']
feedback = fixture['joint_feedback']
wanted_rows = [feedback[k] for k in ('seed', 'accepted_before', 'accepted_after')]
wanted_rows += feedback['transitions']
wanted_rows += list(feedback['fresh_after_search']['latest_by_topic'].values())
wanted = {(r['topic'], r['receipt_stamp_ns']): r for r in wanted_rows}
seen = set()
qualified = False
bag_path = Path(fixture['record']) / 'keyframes.bag'
with rosbag.Bag(str(bag_path)) as bag:
    for topic, message, receipt in bag.read_messages(
            topics=['/joint_states', '/alicia_d/accepted_joint_states',
                    '/grasp_mode/observe_return_measurement_request'],
            start_time=rospy.Time(1791716942), end_time=rospy.Time(1791716954)):
        if topic == '/grasp_mode/observe_return_measurement_request':
            try:
                qualified |= json.loads(message.data) == fixture['qualified_measurement_request']
            except (ValueError, AttributeError):
                pass
            continue
        key = topic, receipt.to_nsec()
        if key not in wanted:
            continue
        row = wanted[key]
        assert message.header.stamp.to_nsec() == row['source_stamp_ns']
        for field in ('name', 'position', 'velocity', 'effort'):
            assert list(getattr(message, field)) == row[field], (key, field)
        seen.add(key)
assert seen == set(wanted), set(wanted) - seen
assert qualified

suites = []
for name, expected in [('independent_r1', 87), ('independent_r2', 80)]:
    manifest_path = NEW / ('verification/gen15_seed_handoff/' + name + '_manifest.json')
    manifest = json.loads(manifest_path.read_text())
    assert manifest['returncode'] == 0 and manifest['source_unchanged']
    xml_path = manifest_path.with_name(name + '.xml')
    cases = ET.parse(xml_path).getroot().findall('.//testcase')
    assert len(cases) == expected
    assert not any(c.find('failure') is not None or c.find('error') is not None for c in cases)
    for filename, digest in sources.items():
        if filename == 'src/alicia_grasp_modes/tests/test_reachable_observation_integration.py' and name == 'independent_r1':
            # Between independent runs, author clarified the test name/comment.
            continue
        assert manifest['source_sha256'][filename] == digest, (name, filename)
    suites.append(dict(manifest=artifact(manifest_path), xml=artifact(xml_path),
        log=artifact(xml_path.with_suffix('.log')), passed=expected,
        note='Scopes overlap; counts are deliberately not added together.'))

report = dict(reviewed=True, blocking_findings=[], reviewer='gen6_contact_search',
    delivery=artifact(delivery_path), source_sha256=sources,
    base_sha256=delivery['base_sha256'], code_scope='Only _joint_space_observation_candidates changes; other module AST nodes are identical.',
    independent_verification=suites,
    exact_record_validation=dict(audit=artifact(audit_path), fixture=artifact(fixture_path),
        bag_path=str(bag_path), exact_feedback_packets=len(seen), exact_qualified_measurement_request=True,
        frozen_source_audit_equal=True, recorded_generator_audit_equal=True),
    findings_closed=[
        'One SDK count plus scale-dependent machine roundoff retains proposals without rounding coordinates or increasing tracking/station thresholds.',
        'Cache hits preserve the original search seed; a series of small updates cannot ratchet the same proposal anchor.',
        'Source or owner invalidation tombstones the prepared object. Manual/epoch/session/selection changes cannot be cleared by a second call.',
        'Generation and cache-hit handoffs repeat source/key/owner validation and require finite unique named feedback no older than 0.5 s.',
        'Actual larger motion or configuration changes regenerate only within the original shared remaining two-second budget. Old candidates are removed on rejection.',
        'The generated endpoint q, target transforms, frozen source and original endpoint evidence are unchanged. Only non-authoritative handoff diagnostics are appended.',
        'The initially proposed extra current-start 15 mm gate was removed. The existing frozen-source baseline and all subsequent gates remain unchanged.',
        'MoveIt observation strict planning calls get_current_state and set_start_state for every invocation; retained proposal seed is neither trajectory start nor execution authority.',
        'A controlled full-path collision still rejects the retained recorded-feedback proposal in the real ModeAware strict loop and publishes no valid execution plan.',
        'Rejected proposals persist truthful input-change reasons in their cached audit rather than exposing the earlier solver status alone.'
    ], limitations=[
        'The discarded GEN15 FK goal was not recorded. The transport regression uses exact GEN15 feedback with an explicitly separate physical endpoint fixture.',
        'Tests do not establish a reachable full-arm path or successful hardware grasp. Strict path, following/support and arrival checks remain required.',
        'No live ROS master/service, robot motion, temperature query or production source edit was performed by this review.'
    ])
out = NEW / 'gen15_analysis/seed_handoff_independent_review.json'
assert Path('/media/zhuyupei/SanStand').is_mount() and not out.exists()
out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(dict(review=artifact(out), blocking_findings=[], feedback_packets=len(seen))))
