#!/usr/bin/python3
"""Read-only typed supplement to the complete native full-stack acceptance.

No import-time ROS work. On explicit execution, run native.accept unchanged on
the exact already accepted instance, then observe pinned artifacts and initial
typed state. The only invoked typed operation is QUERY. Service handshake
probes and QUERY cannot create a lease, change mode, send a target or clean one.
"""
from collections import deque
from pathlib import Path
import argparse
import hashlib
import json
import math
import os
import sys
import threading
import time
import types

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(W/'tools'))
from experiment_storage import select as select_storage

BASELINE_ASSETS_SHA256 = '3abf7c5d740b48dc6ae7577067bd3fb1b7a5f2d32b4fea2f1b09120c96232dda'
ENTRY_SOURCE = 'tools/diagnostics/verify_physical_reference_startup.py'
STORAGE_SOURCE = 'tools/experiment_storage.py'
NATIVE_SOURCE = 'tools/full_stack_start.py'
LAUNCH_SOURCE = '.runtime/start_full_latest_gen36_20261016.sh'
LAUNCH_XML_SOURCE = '.runtime/full_latest_gen36_single_observation_20261016.launch'
R26_PREVIOUS_ASSETS_SHA256 = '16274da520ccd9efc35feae8bd717e2318304a85e08c7c9ac62fcd581dd6fe82'
R26_BASE_RELEASE_SHA256 = '71e79ddb69b169fea1506e263289e8d1de6858966e11c4fa80ed24886c7d7d52'
INHERITED_TYPED_PARENT_SHA256 = '84c223af73021caf9a431cb11ac3d3f73e7059846c399933aadda23cfbedfc84'
R26_TYPED_CHANGED_SOURCES = frozenset((
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/physical_reference_client.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/mapped_contact_execution.py',
    ENTRY_SOURCE))
R26_FULL_CHANGED_SOURCES = R26_TYPED_CHANGED_SOURCES | frozenset((
    'src/alicia_flexible_grasp_supervisor/tests/test_r22_typed_client_independent.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_r22_mapped_episode_independent.py',
    'tools/tests/test_physical_reference_startup.py'))
STORAGE_LAUNCH_PARAMETERS = (
    ('/grasp_6d/remote/gate_audit_output_path',
     '$(env ALICIA_SESSION_LOG_DIR)/planning_audits/grasp6d_gate_audit_latest.json'),
    ('/mujoco_digital_twin/audit_output_path',
     '$(env ALICIA_SESSION_LOG_DIR)/planning_audits/grasp6d_mujoco_audit_latest.json'))
ARM = tuple('Joint%d' % i for i in range(1, 7))
DRIVER = '/alicia_d_driver_node'
HW = '/bessica_d_hw_interface'
SERVICES = {'/alicia_d/physical_reference': DRIVER,
            '/bessica_d_hw_interface/physical_reference': HW}
EMPTY_FIELDS = ('binding_id', 'previous_binding_id', 'owner_token', 'goal_token',
    'episode_id', 'action_goal_id', 'plan_sha256', 'model_sha256', 'proof_sha256',
    'physical_trajectory_sha256', 'wire_trajectory_sha256')
ZERO_FIELDS = ('action_goal_stamp_ns', 'action_transport_sequence', 'selection_generation',
    'request_sequence', 'last_command_sequence', 'prepared_ns', 'deadline_ns',
    'episode_deadline_ns', 'monotonic_remaining_sec', 'outer_monotonic_remaining_sec',
    'stop_duration_sec', 'goal_status', 'goal_error_code')


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    return dict(path=str(Path(path).resolve()), sha256=sha(path))


def pinned(reference):
    raw = Path(reference['path']).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == reference['sha256'], 'pinned evidence changed: '+reference['path'])
    return json.loads(raw)


def admit_output(contract, destination):
    """Bind all audit writes to the accepted session's exact selected disk."""
    binding = contract['storage']
    storage = select_storage(binding['backend'])
    session = Path(contract['output_root']).resolve()
    storage.validate(session, subtree=storage.ros_log_root, expected=binding)
    destination = Path(destination).resolve()
    require(destination.parent == session and not destination.exists(),
            'new native child destination required')
    storage.validate(destination, subtree=storage.ros_log_root, expected=binding)
    return storage, binding, session, destination


def save(path, value, storage, binding):
    path = Path(path).resolve()
    storage.validate(path, subtree=storage.ros_log_root, expected=binding)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(path.read_bytes() == raw, 'audit readback changed')


def native_module(assets):
    path = W/NATIVE_SOURCE; raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == assets['source_sha256'][NATIVE_SOURCE],
            'native verifier changed')
    module = types.ModuleType('_exact_native_acceptance'); module.__file__ = str(path)
    exec(compile(raw, str(path), 'exec'), module.__dict__)
    return module


def verify_storage_launch_derivation(assets, baseline):
    """Admit exactly two appended audit params, preserving every other byte."""
    original_digest = baseline['source_sha256'][LAUNCH_XML_SOURCE]
    artifact_name = str(W/LAUNCH_XML_SOURCE)
    require(baseline['current_sha256'][artifact_name] == original_digest,
            'baseline launch source/artifact binding differs')
    reference = assets['baseline_launch']
    original = Path(reference['path']).read_bytes()
    require(reference['sha256'] == original_digest
            and hashlib.sha256(original).hexdigest() == original_digest,
            'baseline launch bytes changed')
    closing = b'</launch>\n'
    require(original.endswith(closing) and original.count(b'</launch>') == 1,
            'baseline launch has an unsupported closing structure')
    require(all(name.encode() not in original for name, _ in STORAGE_LAUNCH_PARAMETERS),
            'baseline launch already contains audit output parameters')
    inserted = ''.join('  <param name="%s" value="%s"/>\n' % pair
                       for pair in STORAGE_LAUNCH_PARAMETERS).encode()
    current = (W/LAUNCH_XML_SOURCE).read_bytes()
    require(current == original[:-len(closing)] + inserted + closing,
            'launch change is not exactly the two reviewed audit output parameters')
    current_digest = hashlib.sha256(current).hexdigest()
    require(assets['source_sha256'][LAUNCH_XML_SOURCE] == current_digest
            and assets['current_sha256'][artifact_name] == current_digest,
            'current launch source/artifact binding differs')


def r26_source_delta(before, after):
    return {path: dict(before_sha256=before.get(path), after_sha256=after.get(path))
            for path in sorted(set(before) | set(after)) if before.get(path) != after.get(path)}


def verify_r26_case_continuity(previous, current, review):
    """Retain old bodies/skips in addition to native's exact current checks."""
    old_ids, new_ids = set(previous['expected_case_ids']), set(current['expected_case_ids'])
    renames = review.get('reviewed_case_renames')
    require(type(renames) is list and all(type(row) is dict
        and set(row) == {'old_nodeid', 'new_nodeid', 'reason'}
        and all(type(value) is str and value for value in row.values()) for row in renames),
        'r26 explicit case rename mapping required')
    old_names = [row['old_nodeid'] for row in renames]
    new_names = [row['new_nodeid'] for row in renames]
    require(len(old_names) == len(set(old_names)) and len(new_names) == len(set(new_names))
            and set(old_names) == old_ids-new_ids and set(new_names) <= new_ids-old_ids,
            'r26 previous case coverage removed or renamed without exact mapping')
    require(review.get('added_case_ids') == sorted(new_ids-old_ids-set(new_names)),
            'r26 added case inventory differs')
    reverse = dict(zip(new_names, old_names))
    old_cases = {row['nodeid']: row for row in previous['case_results']}
    new_cases = {row['nodeid']: row for row in current['case_results']}
    for nodeid in current['skipped_cases']:
        original = reverse.get(nodeid, nodeid)
        require(original in previous['skipped_cases'] and original in old_cases and nodeid in new_cases,
                'r26 introduced a new skipped body')
        require(all(old_cases[original].get(key) == new_cases[nodeid].get(key)
                    for key in ('setup', 'call', 'teardown', 'skip_reason')),
                'r26 retained skip phase/reason changed')
    require(current['excluded_files'] == previous['excluded_files'], 'r26 exclusion policy changed')


def verify_r26_python_derivation(assets):
    """Exact Python admission-order repair; compiled/runtime contract unchanged."""
    require(assets.get('schema') == 5
            and assets.get('kind') == 'immutable_physical_reference_startup_identity'
            and assets.get('revision') == 'r26_physical_source_admission_python_v1',
            'unsupported r26 Python typed identity')
    require(all(assets.get(key) is False for key in ('runtime_loaded', 'physical_grasp_verified',
                                                   'automatic_execution_authorized')),
            'r26 offline assets cannot claim runtime/physical/automatic acceptance')
    derivation = assets['python_source_derivation']
    previous_ref = derivation['previous_assets']
    base_ref = derivation['base_release']
    require(previous_ref['sha256'] == R26_PREVIOUS_ASSETS_SHA256
            and base_ref['sha256'] == R26_BASE_RELEASE_SHA256,
            'r26 immutable r24 parent differs')
    previous, base = pinned(previous_ref), pinned(base_ref)
    for key in ('expected_nodes', 'topics', 'type_md5', 'required_parameters', 'memory_attestation',
                'formal_build', 'checklist', 'baseline_assets', 'baseline_launch', 'post_build_source_updates'):
        require(assets[key] == previous[key], 'r26 immutable contract/evidence changed: '+key)
    require(assets['baseline_assets']['sha256'] == BASELINE_ASSETS_SHA256,
            'r26 original typed baseline differs')
    verify_storage_launch_derivation(assets, pinned(assets['baseline_assets']))
    require(assets['current_sha256'] == previous['current_sha256'],
            'r26 Python-only repair changed a compiled/generated/model/startup artifact')
    sources = assets['source_sha256']
    typed_delta = r26_source_delta(previous['source_sha256'], sources)
    require(set(sources) == set(previous['source_sha256'])
            and set(typed_delta) == R26_TYPED_CHANGED_SOURCES
            and derivation.get('typed_source_delta') == typed_delta,
            'r26 typed source inventory/delta is not exactly the three reviewed Python sources')
    run = pinned(assets['actual_regression'])
    require(run.get('passed') is True and run.get('complete_scope_requested') is True
            and run.get('offline_scope_complete') is True and run.get('source_unchanged') is True
            and all(run.get(key) == [] for key in ('changed_sources', 'external_boundary_attempts',
                                                  'bad_files', 'duplicate_cases')),
            'r26 completed unchanged full offline result required')
    full_sources = run['source_sha256']
    full_delta = r26_source_delta(base['source_sha256'], full_sources)
    require(set(full_delta) == R26_FULL_CHANGED_SOURCES
            and all(row['before_sha256'] is not None and row['after_sha256'] is not None
                    for row in full_delta.values()),
            'r26 full source delta is not exactly the six reviewed existing files')
    require(all(full_sources.get(path) == digest for path, digest in sources.items())
            and full_sources.get(LAUNCH_SOURCE) == assets['current_sha256'][str(W/LAUNCH_SOURCE)],
            'r26 typed/launch source differs from new full regression')
    review = pinned(derivation['change_review'])
    require(review.get('reviewed') is True
            and review.get('source_scope') == 'r26_physical_source_admission_python_only'
            and review.get('base_release') == base_ref
            and review.get('base_assets') == previous_ref
            and review.get('actual_regression') == assets['actual_regression']
            and review.get('formal_build') == assets['formal_build']
            and review.get('source_delta') == full_delta == derivation.get('source_delta')
            and review.get('typed_runtime_contract_unchanged') is True
            and review.get('formal_cpp_build_unchanged') is True
            and review.get('typed_artifacts_unchanged') is True,
            'r26 exact root Python/source/full review required')
    build = pinned(assets['formal_build'])
    require(type(build.get('returncode')) is int and build['returncode'] == 0
            and build.get('formal_release_build') is True
            and all(base['source_sha256'].get(path) == digest == full_sources.get(path)
                    for path, digest in build['source_sha256'].items()),
            'r26 reused formal C++ build/source identity differs')
    verify_r26_case_continuity(pinned(base['actual_regression']), run, review)
    # Reuse the original pure current-discovery/phase/source/skip validator and
    # reviewed build selector.  No plan/start/accept/configuration call occurs.
    native = native_module(assets)
    release = dict(base, source_sha256=full_sources, source_count=len(full_sources),
                   actual_regression=assets['actual_regression'], driver_build=assets['formal_build'],
                   full_regression_passed=run['full_regression_passed'])
    native.release_evidence(release)
    selected, _ = native.driver_build_evidence(release)
    require(selected == assets['formal_build'], 'r26 native selected another formal build')


def verify_derivation(assets):
    if assets.get('schema') == 6:
        return verify_inherited_release_derivation(assets)
    if assets.get('schema') == 5:
        return verify_r26_python_derivation(assets)
    require(assets.get('schema') == 3
            and assets.get('kind') == 'immutable_physical_reference_startup_identity',
            'unsupported typed startup identity')
    baseline_ref = assets['baseline_assets']
    require(baseline_ref['sha256'] == BASELINE_ASSETS_SHA256, 'typed baseline differs')
    baseline = pinned(baseline_ref)
    for key in ('expected_nodes', 'topics', 'type_md5', 'required_parameters', 'memory_attestation'):
        require(assets[key] == baseline[key], 'typed runtime contract changed: '+key)
    require(assets['formal_build']['sha256'] == baseline['formal_build']['sha256']
            and assets['checklist']['sha256'] == baseline['checklist']['sha256'],
            'typed build/checklist changed')
    original_sources = baseline['source_sha256']; current_sources = assets['source_sha256']
    require(set(current_sources) == set(original_sources) | {ENTRY_SOURCE, STORAGE_SOURCE},
            'typed source inventory differs')
    for name, digest in original_sources.items():
        if name not in (NATIVE_SOURCE, LAUNCH_XML_SOURCE):
            require(current_sources[name] == digest, 'typed production source changed: '+name)
    original_artifacts = baseline['current_sha256']; current_artifacts = assets['current_sha256']
    require(set(current_artifacts) == set(original_artifacts), 'typed artifact inventory differs')
    for name, digest in original_artifacts.items():
        if name not in (str(W/NATIVE_SOURCE), str(W/LAUNCH_SOURCE), str(W/LAUNCH_XML_SOURCE)):
            require(current_artifacts[name] == digest, 'typed compiled/model artifact changed: '+name)
    require(current_artifacts[str(W/NATIVE_SOURCE)] == current_sources[NATIVE_SOURCE],
            'native source/artifact binding differs')
    verify_storage_launch_derivation(assets, baseline)
    regression = pinned(assets['actual_regression'])
    require(regression.get('passed') is True and regression.get('offline_scope_complete') is True
            and regression.get('source_unchanged') is True
            and regression.get('external_boundary_attempts') == [], 'typed current offline evidence incomplete')
    require(all(regression['source_sha256'].get(name) == digest
                for name, digest in current_sources.items()), 'typed sources differ from current regression')
    require(regression['source_sha256'].get(LAUNCH_SOURCE) == current_artifacts[str(W/LAUNCH_SOURCE)],
            'typed launch differs from current regression')


def verify_inherited_release_derivation(assets):
    """Retain typed hardware semantics while admitting reviewed verifier changes."""
    require(assets.get('kind') == 'immutable_physical_reference_startup_identity'
            and assets.get('revision') == 'inherited_manual_restart_verifiers_v1'
            and 'actual_regression' not in assets,
            'inherited typed assets must not invent a current full regression')
    require(all(assets.get(key) is False for key in ('runtime_loaded', 'physical_grasp_verified',
                                                   'automatic_execution_authorized')),
            'inherited offline assets cannot claim runtime/physical/automatic acceptance')
    derivation = assets['inherited_release_derivation']
    require(derivation['previous_assets']['sha256'] == INHERITED_TYPED_PARENT_SHA256,
            'inherited reviewed typed parent differs')
    previous = pinned(derivation['previous_assets'])
    require(previous.get('schema') == 5
            and previous.get('kind') == 'immutable_physical_reference_startup_identity'
            and previous.get('revision') == 'r26_physical_source_admission_python_v1',
            'inherited typed parent must be the prior reviewed Python identity')
    for key in ('expected_nodes', 'topics', 'type_md5', 'required_parameters', 'memory_attestation',
                'formal_build', 'checklist', 'baseline_assets', 'baseline_launch', 'post_build_source_updates'):
        require(assets[key] == previous[key], 'inherited typed runtime/build contract changed: '+key)
    require(assets['baseline_assets']['sha256'] == BASELINE_ASSETS_SHA256,
            'inherited original typed baseline differs')
    verify_storage_launch_derivation(assets, pinned(assets['baseline_assets']))
    release = pinned(assets['current_release'])
    require(release.get('kind') == 'inherited_full_plus_targeted' and 'actual_regression' not in release
            and derivation.get('baseline_full_regression') == release['baseline_full_regression'],
            'typed assets are not bound to the actual inherited release')
    baseline = pinned(release['baseline_full_regression'])
    before, after = previous['source_sha256'], assets['source_sha256']
    require(set(before) == set(after)
            and all(baseline['source_sha256'].get(p) == h for p, h in before.items()),
            'prior typed sources differ from the recorded full baseline')
    delta = r26_source_delta(before, after)
    require(set(delta) == {NATIVE_SOURCE, ENTRY_SOURCE}
            and derivation.get('typed_source_delta') == delta
            and all(release['source_sha256'].get(p) == h for p, h in after.items()),
            'typed source delta is not exactly the reviewed native/typed verifiers')
    old_artifacts, artifacts = previous['current_sha256'], assets['current_sha256']
    artifact_delta = r26_source_delta(old_artifacts, artifacts)
    native_path = str(W/NATIVE_SOURCE)
    require(set(old_artifacts) == set(artifacts) and set(artifact_delta) == {native_path}
            and derivation.get('artifact_delta') == artifact_delta
            and old_artifacts[native_path] == before[NATIVE_SOURCE]
            and artifacts[native_path] == after[NATIVE_SOURCE],
            'inherited typed compiled/generated/model/launch artifacts changed')
    native = native_module(assets)
    native.release_evidence(release)
    selected, build = native.driver_build_evidence(release)
    require(selected == assets['formal_build'] and build.get('formal_release_build') is True,
            'inherited release must select the unchanged formal build')


def verify_accepted_release_binding(assets, contract, release):
    if assets.get('schema') == 6:
        require(assets['current_release'] == contract['release']
                and pinned(assets['current_release']) == release,
                'typed identity is not bound to the accepted inherited release')
    else:
        require(release.get('actual_regression') == assets['actual_regression'],
                'typed identity is not bound to the accepted current regression')


def verify_assets(assets):
    verify_derivation(assets)
    pinned(assets['formal_build']); pinned(assets['checklist'])
    for update in assets['post_build_source_updates']:
        targeted = pinned(update['targeted_validation']); pinned(update['review'])
        require(sha(update['patch']['path']) == update['patch']['sha256'], 'recorder patch changed')
        require(targeted.get('passed') is True
                and targeted['actual_project_source_sha256'].get(str(W/update['path'])) == update['after_sha256']
                and assets['source_sha256'].get(update['path']) == update['after_sha256'],
                'post-build recorder validation is not bound to current source')
    for relative, digest in assets['source_sha256'].items():
        require(not Path(relative).is_absolute() and '..' not in Path(relative).parts, 'invalid relative build source')
        require(sha(W/relative) == digest, 'frozen r22 source changed: '+relative)
    for path, digest in assets['current_sha256'].items():
        require(W in Path(path).parents and sha(path) == digest, 'frozen r22 artifact changed: '+path)


def runtime_binaries(accepted, assets):
    result = {}
    for name, relative in ((DRIVER, 'devel/lib/alicia_d_driver/alicia_d_driver_node'),
                           (HW, 'devel/lib/alicia_d_driver/alicia_d_hw_interface_node')):
        pid = accepted['nodes'][name]['pid']; expected = W/relative
        proc = Path('/proc')/str(pid)/'exe'
        require(proc.resolve() == expected.resolve(), 'running executable path differs: '+name)
        require(sha(proc) == assets['current_sha256'][str(expected)], 'running executable SHA differs: '+name)
        result[name] = dict(pid=pid, exe_path=str(proc.resolve()), exe_sha256=sha(proc))
    library = W/'devel/lib/libalicia_d_hw_interface.so'; stat = library.stat()
    require(sha(library) == assets['current_sha256'][str(library)], 'HW library file SHA differs')
    rows = []
    for line in (Path('/proc')/str(accepted['nodes'][HW]['pid'])/'maps').read_text().splitlines():
        fields = line.split(None, 5)
        if len(fields) != 6 or 'libalicia_d_hw_interface.so' not in fields[5]:
            continue
        address, perms, offset, device, inode, path = fields
        major, minor = (int(part, 16) for part in device.split(':'))
        require(path == str(library) and not path.endswith(' (deleted)'), 'different/deleted HW library mapped')
        require((major, minor) == (os.major(stat.st_dev), os.minor(stat.st_dev))
                and int(inode) == stat.st_ino, 'HW library mapped device/inode differs')
        rows.append(dict(address=address, permissions=perms, offset=offset,
                         device=device, inode=int(inode), path=path))
    require(rows and any('x' in row['permissions'] for row in rows), 'HW executable library mapping absent')
    again = library.stat()
    require((stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns) ==
            (again.st_dev, again.st_ino, again.st_size, again.st_mtime_ns), 'HW library changed while inspecting maps')
    result['hardware_library'] = dict(path=str(library), sha256=sha(library), maps=rows)
    result['memory_attestation'] = False
    return result


def graph_bindings(master, assets, probe=False):
    publishers, subscribers, services = (dict(rows) for rows in master.getSystemState())
    topic_types = dict(master.getTopicTypes())
    required = {name: (row['publisher'], row['type']) for name, row in assets['topics'].items()}
    required.update({'/alicia_d/sdk_command': (DRIVER, 'sensor_msgs/JointState'),
                     '/alicia_d/accepted_joint_states': (DRIVER, 'sensor_msgs/JointState'),
                     '/alicia_d/control_reference': (DRIVER, 'sensor_msgs/JointState'),
                     '/alicia_controller/state': (HW, 'control_msgs/JointTrajectoryControllerState')})
    for name, (owner, kind) in required.items():
        require(publishers.get(name) == [owner] and topic_types.get(name) == kind,
                'typed/raw publisher or type differs: '+name)
    require(subscribers.get('/alicia_d/physical_reference_command') == [DRIVER],
            'mapped command subscriber differs')
    result = dict(topics={name: dict(publisher=owner, type=kind) for name, (owner, kind) in required.items()}, services={})
    if probe:
        import rosservice
    for name, owner in SERVICES.items():
        require(services.get(name) == [owner], 'typed service owner differs: '+name)
        uri = master.lookupService(name); row = dict(owner=owner, uri=uri)
        if probe:
            headers = rosservice.get_service_headers(name, uri)  # TCPROS probe=1; no service request.
            require(headers.get('callerid') == owner
                    and headers.get('type') == 'alicia_d_driver/SetPhysicalReference'
                    and headers.get('md5sum') == assets['type_md5']['alicia_d_driver/SetPhysicalReference'],
                    'typed service handshake differs: '+name)
            row['handshake'] = {key: headers.get(key) for key in ('callerid', 'type', 'md5sum')}
        result['services'][name] = row
    return result


def fresh(stamp, now, epoch):
    return type(stamp) is int and 0 < epoch <= stamp <= now and now-stamp <= 500000000


def positions(message):
    names = list(message.name)
    require(len(names) == len(set(names)) == len(message.position) and set(ARM) <= set(names), 'joint layout invalid')
    values = [float(message.position[names.index(name)]) for name in ARM]
    require(all(math.isfinite(v) for v in values), 'nonfinite joint position')
    return values


def words(values):
    require(len(values) == 6 and all(math.isfinite(q) and -math.pi-1e-12 <= q <= math.pi-2*math.pi/4096+1e-12 for q in values),
            'SDK position domain invalid')
    return tuple(int(math.floor((q*180/math.pi+180)/360*4096+.5)) for q in values)


def decoded(counts):
    require(len(counts) == 6 and all(type(c) is int and 0 <= c <= 4095 for c in counts), 'SDK words invalid')
    return [(-180+c/4096*360)*math.pi/180 for c in counts]


class Observer:
    def __init__(self, rospy):
        from sensor_msgs.msg import JointState
        from control_msgs.msg import JointTrajectoryControllerState
        from alicia_d_driver.msg import PhysicalReferenceState
        self.lock = threading.Lock(); self.rows = {}; self.subscribers = []
        topics = [('sdk', '/alicia_d/sdk_command', JointState),
                  ('encoder', '/alicia_d/accepted_joint_states', JointState),
                  ('reference', '/alicia_d/control_reference', JointState),
                  ('controller', '/alicia_controller/state', JointTrajectoryControllerState),
                  ('driver', '/alicia_d/physical_reference_state', PhysicalReferenceState),
                  ('hardware', '/bessica_d_hw_interface/physical_reference_state', PhysicalReferenceState)]
        for key, topic, kind in topics:
            self.rows[key] = deque(maxlen=256)
            self.subscribers.append(rospy.Subscriber(topic, kind,
                lambda message, k=key: self.receive(k, message), queue_size=100))

    def receive(self, key, message):
        # Do not filter reference modes: a later nonmanual message replaces the
        # preceding sdk_manual claim even when that old message was fresh.
        with self.lock:
            self.rows[key].append(message)

    def snapshot(self):
        with self.lock:
            return {key: list(rows) for key, rows in self.rows.items()}

    def close(self):
        for handle in self.subscribers:
            handle.unregister()


def validate_state(state, epoch, now, rows, assets, owner=None):
    if owner is not None:
        header = state._connection_header
        require(header.get('callerid') == owner and header.get('latching') == '0'
                and header.get('md5sum') == assets['type_md5']['alicia_d_driver/PhysicalReferenceState'],
                'fresh state transport owner/latch/MD5 invalid')
    require(state.header.frame_id == 'physical_reference_v1' and fresh(state.header.stamp.to_nsec(), now, epoch), 'typed state stale')
    require(state.phase == 0 and state.epoch_ns == epoch and all(getattr(state, key) == '' for key in EMPTY_FIELDS)
            and all(getattr(state, key) == 0 for key in ZERO_FIELDS), 'initial typed binding is not empty LEGACY')
    require(len(state.bias_rad) == 6 and all(x == 0. for x in state.bias_rad)
            and state.goal_observed is False and state.goal_terminal is False and state.output_gated is False,
            'initial typed goal/map state differs')
    require(all(getattr(state, name) is True for name in ('evidence_valid', 'pending_complete', 'stationary', 'manual_reference')),
            'typed_not_ready: fresh confirmed pending/stationary/manual evidence absent')
    for key, field in (('sdk', 'sdk_stamp_ns'), ('encoder', 'encoder_stamp_ns'), ('controller', 'controller_stamp_ns')):
        stamp = getattr(state, field)
        require(fresh(stamp, now, epoch) and any(m.header.stamp.to_nsec() == stamp for m in rows[key]), 'typed source stamp is stale/unseen: '+key)
    counts = list(state.sdk_counts); expected = decoded(counts)
    require(all(math.isfinite(v) and abs(v-q) <= 1e-12 for v, q in zip(state.wire_positions_rad, expected))
            and len(state.wire_positions_rad) == 6, 'typed wire words/decoded values differ')
    controller = next(m for m in rows['controller'] if m.header.stamp.to_nsec() == state.controller_stamp_ns)
    desired = controller.desired
    require(len(controller.joint_names) == len(set(controller.joint_names)) == 6 and set(controller.joint_names) == set(ARM), 'JTC joint layout differs')
    require(all(len(getattr(desired, key)) == 6 for key in ('positions', 'velocities', 'accelerations')), 'JTC desired vectors missing')
    ordered = [desired.positions[list(controller.joint_names).index(name)] for name in ARM]
    require(len(state.physical_reference_rad) == 6 and all(math.isfinite(v) and abs(v-q) <= 1e-12 for v, q in zip(state.physical_reference_rad, ordered)),
            'LEGACY physical_reference_rad does not match actual JTC desired')
    require(all(math.isfinite(v) and v == 0. for values in (desired.velocities, desired.accelerations) for v in values), 'JTC is not zero velocity/acceleration')
    return dict(phase=state.phase, code=state.code, epoch_ns=epoch, sdk_counts=counts,
                state_stamp_ns=state.header.stamp.to_nsec(), sdk_stamp_ns=state.sdk_stamp_ns,
                encoder_stamp_ns=state.encoder_stamp_ns, controller_stamp_ns=state.controller_stamp_ns,
                request_sequence=state.request_sequence, last_command_sequence=state.last_command_sequence,
                manual_reference=True, evidence_valid=True, pending_complete=True, stationary=True,
                physical_reference_rad=list(state.physical_reference_rad), wire_positions_rad=list(state.wire_positions_rad))


def observed_ready(observer, epoch, rospy, assets, query_states=None):
    rows = observer.snapshot(); require(all(rows.values()), 'typed/raw telemetry pending')
    now = rospy.Time.now().to_nsec(); require(epoch > 0, 'positive current epoch required')
    for key in ('sdk', 'encoder', 'reference', 'controller'):
        message = rows[key][-1]; owner = HW if key == 'controller' else DRIVER
        require(message._connection_header.get('callerid') == owner and fresh(message.header.stamp.to_nsec(), now, epoch), 'raw source owner/stamp differs: '+key)
    sdk, encoder, reference = (rows[key][-1] for key in ('sdk', 'encoder', 'reference'))
    require(sdk.header.frame_id == 'sdk_transmitted' and encoder.header.frame_id == 'sdk_measured', 'real SDK frame kinds differ')
    counts = words(positions(sdk))
    require(reference._connection_header.get('latching') == '0' and reference.header.frame_id == 'sdk_manual'
            and reference.header.stamp == sdk.header.stamp and words(positions(reference)) == counts,
            'newest actual control_reference is not paired sdk_manual')
    windows = {}
    for key in ('sdk', 'encoder'):
        last = rows[key][-1].header.stamp.to_nsec()
        window = [m for m in rows[key] if last-500000000 <= m.header.stamp.to_nsec() <= last]
        stamps = [m.header.stamp.to_nsec() for m in window]
        require(len(window) >= 3 and stamps[-1]-stamps[0] >= 300000000
                and all(a < b and b-a <= 250000000 for a, b in zip(stamps, stamps[1:])), 'stationary source window incomplete: '+key)
        require(all(m._connection_header.get('callerid') == DRIVER and m.header.frame_id == ('sdk_transmitted' if key == 'sdk' else 'sdk_measured') for m in window), 'stationary window source changed')
        q = [positions(m) for m in window]
        if key == 'sdk':
            require(all(words(row) == counts for row in q), 'successful SDK words changed during stationary window')
        else:
            require(all(max(row[j] for row in q)-min(row[j] for row in q) <= 2*math.pi/4096+1e-12 for j in range(6)), 'real encoder window is not stationary')
        windows[key] = dict(samples=len(window), start_ns=stamps[0], end_ns=stamps[-1])
    states = {}
    for key, owner in (('driver', DRIVER), ('hardware', HW)):
        states[key] = validate_state(rows[key][-1], epoch, now, rows, assets, owner)
        require(tuple(states[key]['sdk_counts']) == counts, 'typed endpoint disagrees with successful SDK words')
    queried = {}
    for key, state in (query_states or {}).items():
        queried[key] = validate_state(state, epoch, now, rows, assets)
        require(tuple(queried[key]['sdk_counts']) == counts, 'QUERY state differs from successful SDK words')
    return dict(states=states, query_states=queried, source_windows=windows,
                sdk_counts=list(counts), accepted_positions_rad=positions(encoder),
                newest_reference_frame=reference.header.frame_id,
                newest_reference_stamp_ns=reference.header.stamp.to_nsec(), observed_ns=now)


def bounded_query(proxy, request):
    done = threading.Event(); box = {}
    def call():
        try: box['response'] = proxy(request)
        except BaseException as exc: box['error'] = repr(exc)
        finally: done.set()
    threading.Thread(target=call, daemon=True).start()
    require(done.wait(3.), 'read-only typed QUERY timed out; no automatic mutation or retry')
    require('error' not in box, 'read-only typed QUERY failed: '+str(box.get('error')))
    return box['response']


def wait_for_fixed_query_sources(observer, epoch, rospy, assets, query_states, audit):
    """Wait only for messages named by the two already returned QUERY states.

    Separate TCPROS connections can deliver a service receipt before the raw
    telemetry it references. Neither QUERY response nor any deadline is renewed.
    Once those exact messages arrive, every original readiness gate runs again.
    """
    require(set(query_states) == {'driver', 'hardware'}, 'both fixed QUERY receipts required')
    fields = (('sdk', 'sdk_stamp_ns'), ('encoder', 'encoder_stamp_ns'),
              ('controller', 'controller_stamp_ns'))
    fixed = {}
    for key, state in query_states.items():
        require(state.epoch_ns == epoch and epoch > 0, 'QUERY epoch differs before source wait')
        fixed[key] = dict(state_stamp_ns=state.header.stamp.to_nsec(),
                          **{field: getattr(state, field) for _source, field in fields})
    started_ros_ns = rospy.Time.now().to_nsec()
    started_monotonic = time.monotonic()
    stamps = [stamp for row in fixed.values() for stamp in row.values()]
    require(all(fresh(stamp, started_ros_ns, epoch) for stamp in stamps),
            'fixed QUERY receipt/source already stale or invalid; no retry')
    expires_ros_ns = min(stamps)+500000000
    expires_monotonic = started_monotonic+(expires_ros_ns-started_ros_ns)*1e-9
    audit.update(fixed_query_stamps=fixed, epoch_ns=epoch,
                 started_ros_ns=started_ros_ns, expires_ros_ns=expires_ros_ns,
                 original_wait_budget_sec=(expires_ros_ns-started_ros_ns)*1e-9,
                 query_reissued=False, receipt_refreshed=False, snapshots=0)
    try:
        while True:
            now = rospy.Time.now().to_nsec()
            require(started_ros_ns <= now <= expires_ros_ns
                    and time.monotonic() <= expires_monotonic
                    and all(fresh(stamp, now, epoch) for stamp in stamps),
                    'fixed QUERY source wait expired or clock moved backward; no retry')
            rows = observer.snapshot(); audit['snapshots'] += 1
            missing = []
            for key, state in query_states.items():
                require(state.header.stamp.to_nsec() == fixed[key]['state_stamp_ns']
                        and state.epoch_ns == epoch
                        and all(getattr(state, field) == fixed[key][field] for _source, field in fields),
                        'fixed QUERY response changed while awaiting its sources')
                for source, field in fields:
                    if not any(message.header.stamp.to_nsec() == fixed[key][field]
                               for message in rows.get(source, ())):
                        missing.append(dict(endpoint=key, source=source, stamp_ns=fixed[key][field]))
            audit['last_missing'] = missing
            if not missing:
                ready = observed_ready(observer, epoch, rospy, assets, query_states)
                finished_ros_ns = rospy.Time.now().to_nsec()
                require(started_ros_ns <= finished_ros_ns <= expires_ros_ns
                        and time.monotonic() <= expires_monotonic,
                        'fixed QUERY evidence expired during complete readiness check')
                audit['completed'] = True
                return ready
            remaining = min(expires_monotonic-time.monotonic(),
                            (expires_ros_ns-now)*1e-9)
            require(remaining > 0., 'fixed QUERY referenced telemetry did not arrive in its original window')
            time.sleep(min(.01, remaining))
    finally:
        audit['waited_sec'] = time.monotonic()-started_monotonic
        audit['finished_ros_ns'] = rospy.Time.now().to_nsec()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--assets-sha256', required=True)
    parser.add_argument('--acceptance', type=Path, required=True)
    parser.add_argument('--acceptance-sha256', required=True)
    parser.add_argument('--destination', type=Path, required=True,
                        help='new direct child of the original startup session')
    args = parser.parse_args()
    require(__debug__, 'unoptimized Python required')
    assets_ref = dict(path=str(args.assets.resolve()), sha256=args.assets_sha256)
    prior_ref = dict(path=str(args.acceptance.resolve()), sha256=args.acceptance_sha256)
    prior = pinned(prior_ref)
    require(prior.get('passed') is True and prior.get('manual_only') is True
            and prior.get('all_configured_nodes_started') is True and prior.get('automatic_grasp_started') is False,
            'complete prior native manual acceptance required')
    contract_ref = prior['contract']; contract = pinned(contract_ref)
    storage, storage_binding, session, destination = admit_output(contract, args.destination)
    audit = session/('typed_startup_invocation_'+str(time.time_ns())+'.json')
    record = dict(passed=False, started_ns=time.time_ns(), entry=ref(__file__), assets=assets_ref,
        prior_acceptance=prior_ref, destination=str(destination), storage=storage_binding,
        motion_commands_sent=False, mode_commands_sent=False,
        torque_disable_commands_sent=False, temperature_queried=False, services_restarted=False,
        typed_mutation_requests_sent=False, memory_attestation=False, automatic_execution_authorized=False)
    observer = monitor = None; rospy = None; admitted_destination = None
    try:
        assets = pinned(record['assets']); verify_assets(assets)
        native = native_module(assets)
        require(pinned(prior_ref) == prior and pinned(contract_ref) == contract, 'startup evidence changed')
        validated_session, config, release = native.validate_contract(contract)
        require(validated_session == session, 'native session binding differs')
        verify_accepted_release_binding(assets, contract, release)
        require(release.get('driver_build') == assets['formal_build'], 'release does not select corrected r22 formal build')
        require(all(release['source_sha256'].get(p) == h for p, h in assets['source_sha256'].items()), 'r22 sources not bound to current full release')
        storage.validate(destination, subtree=storage.ros_log_root, expected=storage_binding)
        admitted_destination = destination
        require(config['parameters'].get('/grasp/unknown_physical_reference_mapping_enabled') is True, 'startup configuration did not opt into mapping')
        # This is the original complete verifier, not a replacement for its
        # graph, GUI, SDK20, fresh RGBD, remote, source or controller checks.
        native.accept(contract, contract_ref, destination=destination,
                      verifier_revision=ref(__file__), expected_runtime=prior)
        accepted_ref = native.ref(destination/'result.json'); accepted = native.read_ref(accepted_ref)
        record['native_acceptance'] = accepted_ref
        import rospy as ros_api
        import rosgraph
        from alicia_d_driver.msg import PhysicalReferenceState, PhysicalReferenceCommand
        from alicia_d_driver.srv import SetPhysicalReference, SetPhysicalReferenceRequest
        rospy = ros_api
        for kind in (PhysicalReferenceState, PhysicalReferenceCommand, SetPhysicalReference):
            require(kind._md5sum == assets['type_md5'][kind._type], 'imported generated type MD5 differs')
            module_file = Path(sys.modules[kind.__module__].__file__).resolve()
            require(str(module_file) in assets['current_sha256'] and sha(module_file) == assets['current_sha256'][str(module_file)], 'generated binding imported from another workspace')
        master = rosgraph.Master(rospy.get_name())
        record['binaries_before'] = runtime_binaries(accepted, assets)
        record['typed_graph_before'] = graph_bindings(master, assets, probe=True)
        for key, expected in assets['required_parameters'].items():
            value = rospy.get_param(key)
            require(type(value) is bool and value is expected and config['parameters'].get(key) is expected, 'live/frozen parameter differs: '+key)
        monitor = native.Monitor(config); observer = Observer(rospy)
        startup = native.read_ref(native.ref(session/'startup.json'))
        deadline = time.monotonic()+12.; last = None
        while time.monotonic() < deadline:
            try:
                manual = monitor.sample(startup['started_ns'])
                ready = observed_ready(observer, manual['epoch_ns'], rospy, assets); break
            except RuntimeError as exc:
                last = str(exc); time.sleep(.02)
        else: raise RuntimeError('typed_not_ready without corrective action: '+str(last))
        record['observed_initial_contract'] = ready
        query_states = {}
        for (name, owner), key in zip(SERVICES.items(), ('driver', 'hardware')):
            request = SetPhysicalReferenceRequest(); request.operation = request.QUERY
            response = bounded_query(rospy.ServiceProxy(name, SetPhysicalReference), request)
            require(response.success is True and response.code == 'LEGACY', 'read-only QUERY is not initial LEGACY')
            query_states[key] = response.state
        record['query_receipt_source_wait'] = {}
        record['post_query_initial_contract'] = wait_for_fixed_query_sources(
            observer, manual['epoch_ns'], rospy, assets, query_states,
            record['query_receipt_source_wait'])
        verify_assets(assets); native.validate_contract(contract)
        nodes, aliases = native.node_snapshot(master, accepted['core_service'], config, rospy.get_name())
        require(nodes == accepted['nodes'] and aliases == accepted['auxiliary_nodes']
                and native.properties(accepted['core_unit']) == accepted['core_service']
                and native.process(master.getPid()) == accepted['master_identity'], 'native accepted instance changed')
        sim = native.properties(startup['components']['simulation']['unit'])
        require(sim['MainPID'] == str(accepted['simulation']['pid'])
                and sim['InvocationID'] == accepted['simulation']['invocation_id']
                and sim['ControlGroup'] == accepted['simulation']['control_group'], 'simulation instance changed')
        record['binaries_after'] = runtime_binaries(accepted, assets)
        require(record['binaries_after'] == record['binaries_before'], 'running executable/library mapping changed')
        after_graph = graph_bindings(master, assets)
        record['typed_graph_after'] = after_graph
        require(after_graph['topics'] == record['typed_graph_before']['topics']
                and all(after_graph['services'][name]['uri'] == row['uri'] for name, row in record['typed_graph_before']['services'].items()), 'typed owner/URI changed')
        for key, expected in assets['required_parameters'].items():
            require(rospy.get_param(key) is expected, 'live parameter changed before seal: '+key)
        final = monitor.sample(startup['started_ns'])
        require(final['epoch_ns'] == manual['epoch_ns'], 'driver epoch changed during additional verification')
        record['final_native_manual_window'] = final
        record['final_typed_initial_contract'] = observed_ready(observer, final['epoch_ns'], rospy, assets)
        remote = native.read_ref(accepted['remote_dependency'])
        require(time.time_ns()-remote['finished_ns'] < 30000000000, 'native remote health expired during additional checks')
        record.update(passed=True, typed_initial_contract_ready=True,
                      physical_or_geometric_execution_ready=False, physical_grasp_verified=False)
    except BaseException as exc:
        record['error'] = repr(exc)
        raise
    finally:
        errors = []
        for handle in (observer, monitor):
            if handle is not None:
                try: handle.close()
                except BaseException as exc: errors.append(repr(exc))
        if errors:
            record['passed'] = False; record['observer_close_errors'] = errors
        record['finished_ns'] = time.time_ns(); save(audit, record, storage, storage_binding)
        if admitted_destination is not None and admitted_destination.is_dir():
            save(admitted_destination/'physical_reference_startup_result.json', record, storage, storage_binding)
        if rospy is not None:
            rospy.signal_shutdown('read-only typed startup supplement ended')
    require(record['passed'], 'additional verifier did not seal successfully')
    print(json.dumps(dict(passed=True, result=ref(args.destination/'physical_reference_startup_result.json'),
                         native_acceptance=record['native_acceptance'], memory_attestation=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
