#!/usr/bin/python3
"""Accept the already loaded H4 after its recorded legacy trajectory.

This finite continuation has no reload, child, reset or typed mutation entry.
The original H4 runtime, its completed production regression, and the failed
initial acceptance remain immutable. Four new files receive their own exact
isolated targeted admission. The original recorded Session owns any attempt.
"""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import time

W = Path(__file__).resolve().parents[3]
PREFIX = '.runtime/r27_observation_planning_20261102/post_trajectory_acceptance/'
SELF = PREFIX+'adapter.py'
READINESS = PREFIX+'typed_readiness.py'
TESTS = (PREFIX+'tests/test_adapter.py', PREFIX+'tests/test_typed_readiness.py')
NEW_SOURCES = frozenset((SELF, READINESS)+TESTS)
H4_PATH = '.runtime/r27_observation_planning_20261102/runtime_binding/remote_only_revision_r4.py'
H4_SHA = '75a32954dfd1ce19a162df183e236e494266df4a02a2d702189e7c649d4bdbd4'
B = Path('/home/zhuyupei/ssd_data/实验记录/Robot-Controller')
M = B/'maintenance/r27_observation_planning_20261102'
N = M/'new_near_field_failure_20261102'
LOADED_OUTPUT = B/'ros_log/full_latest_r26_manual_20261102_r1/r27_minimum_placement_20261103_r2'
OLD_RECORD = B/'grasp_records/20261103T071544.996136Z_43e3869f5f654e71aadc84817e74c7c0'
ANCHORS = {
    'loaded_revision': dict(path=str(M/'remote_only_prepare_minimum_ray_20261103_r2/revision.json'),
        sha256='f814ff5c7ddf2728fa863cceb5f35fc4f9fe8eae5e24611ae27d430a1a2f535c'),
    'loaded_reload': dict(path=str(LOADED_OUTPUT/'reload_result.json'),
        sha256='b4e012bee1c1c29145ce11897b82d1b3eef6fbc557a9edc641879b9f6dc505d8'),
    'failed_initial_acceptance': dict(path=str(LOADED_OUTPUT/'acceptance/result.json'),
        sha256='371932ab31801d5ebec3cefb0275a0a55a32df176013dc25e96310b4e438fb5f'),
    'snapshot': dict(path=str(LOADED_OUTPUT/'post_attempt_typed_readonly.json'),
        sha256='3fe198949bd645f389c4b2469e301260860957ae0b01873c85b3dec8d23e6dfa'),
    'recorded_history': dict(path=str(N/'typed_attempt1_terminal_review_r1/recorded_typed_transitions.json'),
        sha256='28195d9f9e7fc438e249a77d5f40d2770289f9f3adfbb85c9b0c81c9620a8702'),
    'recorded_actions': dict(path=str(N/'typed_attempt1_terminal_review_r1/recorded_action_goal_terminal_r1.json'),
        sha256='d33c07c03533bef2285a17fa3f7b7c866bb86a99a4e58dd901c373cc99d23806'),
    'epoch_context': dict(path=str(N/'typed_attempt1_terminal_review_r1/post_attempt_epoch_context_r1.json'),
        sha256='813d861e1a3d71cc50683f10d616a27c31694a87111f40d978749f2994ae83f0'),
    'prior_authorization': dict(path=str(OLD_RECORD/'authorization.json'),
        sha256='9b7ad5065ec0dc8725da6779fec2dcd730dba811ad9d3d2956636afc1036ea66'),
    'prior_manifest': dict(path=str(OLD_RECORD/'manifest.json'),
        sha256='0cd04a007eb2db0fc4df95488fbb55119a5167f03b7a4f10971da137ca8d0e3a'),
}
NO_ACTION = dict(cold_check_executed=False, temperature_queried=False,
    torque_disable_commands_sent=False, motion_commands_sent=False,
    mode_commands_sent=False, physical_grasp_verified=False)


def require(value, message):
    if not value:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=sha(path))


def read_ref(reference):
    require(type(reference) is dict and set(reference) == {'path', 'sha256'}
        and Path(reference['path']).is_absolute(), 'exact absolute evidence reference required')
    raw = Path(reference['path']).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == reference['sha256'], 'bound evidence changed: '+reference['path'])
    return json.loads(raw)


def load_module(relative, digest):
    path = W/relative
    require(sha(path) == digest, 'continuation dependency source changed: '+relative)
    name = '_post_trajectory_'+path.stem+'_'+digest[:16]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    # Compile the checked source directly; no package-local bytecode is used.
    exec(compile(path.read_bytes(), str(path), 'exec', dont_inherit=True), module.__dict__)
    return module


def validate_anchor_chain(base, h4, documents):
    """Bind history to the retained processes; never synthesize bag callerids."""
    loaded, failed = documents['loaded_reload'], documents['failed_initial_acceptance']
    h4.validate_reload_chain(ANCHORS['loaded_reload'], loaded, base.reference,
        base.output, loaded['execution_review'])
    read_ref(loaded['execution_review'])
    require(base.reference == ANCHORS['loaded_revision'] and base.output == LOADED_OUTPUT,
            'continuation is for the one already loaded H4 revision')
    require(failed.get('passed') is False and failed.get('kind') == 'r27_remote_only_acceptance_r4'
        and failed['revision'] == base.reference and failed['reload'] == ANCHORS['loaded_reload']
        and failed['execution_review'] == loaded['execution_review']
        and failed['error'] == "RuntimeError('typed_not_ready without corrective action: initial typed binding is not empty LEGACY')"
        and failed.get('automatic_grasp_started') is False, 'failed initial acceptance provenance differs')
    manifest, authorization = documents['prior_manifest'], documents['prior_authorization']
    history, actions = documents['recorded_history'], documents['recorded_actions']
    snapshot, epoch = documents['snapshot'], documents['epoch_context']
    require(manifest['state'] == 'complete' and manifest['recorder_closed'] is True
        and manifest['local_data_deleted'] is False and manifest['result'] == 'failure'
        and authorization['manual_return_verified'] is True
        and authorization['manual_return']['passed'] is True
        and authorization['startup_acceptance'] == h4.PARENT_ACCEPTANCE,
        'closed prior attempt and actual manual return required; held/unknown is not retry authority')
    bag = next(row for row in manifest['files'] if row['path'] == 'keyframes.bag')
    for document in (history, actions):
        require(document['record'] == str(OLD_RECORD)
            and document['manifest_sha256'] == ANCHORS['prior_manifest']['sha256']
            and document['bag_sha256'] == bag['sha256']
            and document.get('source_record_unchanged') is True,
            'history is not from the sealed prior recording')
        stat = (OLD_RECORD/'keyframes.bag').stat()
        require(document['bag_identity'] == [stat.st_size, stat.st_mtime_ns, stat.st_ino]
            and bag['size_bytes'] == stat.st_size, 'recorded bag identity changed')
    for states in (authorization['before']['node_identities'],
                   authorization['manual_return']['final_snapshot']['node_identities']):
        require(all(states[name] == loaded['nodes'][name] for name in loaded['nodes'] if name != h4.REMOTE),
                'historical recorder/control owners differ from retained H4 processes')
    require(snapshot['kind'] == 'current_post_attempt_typed_readonly'
        and snapshot['state']['epoch_ns'] == loaded['state']['epoch_ns']
        and snapshot['state']['manual_mode'] is True
        and snapshot['state']['task_active'] is False and snapshot['state']['actions_active'] is False
        and snapshot['service_query_performed'] is False
        and snapshot['motion_commands_sent'] is False and snapshot['mode_commands_sent'] is False,
        'current post-trajectory snapshot/reload binding differs')
    require(epoch['snapshot'] == ANCHORS['snapshot'] and epoch['recorded_history'] == ANCHORS['recorded_history']
        and epoch['goal_terminal'] == ANCHORS['recorded_actions']
        and epoch['authorization'] == ANCHORS['prior_authorization']
        and epoch['observations']['current_snapshot_epoch_ns'] == loaded['state']['epoch_ns']
        and epoch['observations']['recorded_epoch_ns'] == authorization['manual_return']['final_snapshot']['epoch_ns']
        and epoch['observations']['bag_connection_callerid_available'] is False,
        'historical and current epoch domains must retain their actual evidence')
    require(all(base.sources.get(path) == digest for path, digest in epoch['source_sha256'].items()),
            'epoch interpretation source differs from loaded release')
    require(all(document['storage_binding'] == base.binding for document in (history, actions, snapshot, epoch))
            and manifest['metadata']['storage'] == base.binding, 'history storage binding differs')
    return dict(loaded_reload=ANCHORS['loaded_reload'], failed_initial_acceptance=ANCHORS['failed_initial_acceptance'],
        current_epoch_ns=loaded['state']['epoch_ns'], recorded_epoch_ns=epoch['observations']['recorded_epoch_ns'],
        original_bag_message_callerid_available=False,
        historical_owner_basis='original recorded Session preflight/manual-return process and graph admission',
        same_retained_control_processes=True, old_epoch_claimed_current=False)


def validate_targeted_extension(base, h4, review, targeted_ref):
    """Exact four-file additive scope; original 4282-byte identities unchanged."""
    require(review.get('kind') == 'r27_post_trajectory_continuation_source_review'
        and review.get('reviewed') is True and review.get('anchors') == ANCHORS,
        'exact reviewed post-trajectory continuation required')
    new = review['new_source_sha256']
    require(set(new) == NEW_SOURCES and not set(new).intersection(base.sources),
            'only four new continuation files are permitted')
    require(all(sha(W/path) == digest for path, digest in new.items()), 'new continuation source changed')
    require(review['loaded_release'] == base.data['release']
        and review['production_regression'] == base.release['actual_regression'], 'loaded production release differs')
    wanted = dict(base.sources, **new)
    wrapper_review_ref = review['regression_source_review']
    wrapper_review = read_ref(wrapper_review_ref)
    old_wrapper_review = read_ref(base.review['regression_source_review'])
    require(wrapper_review.get('kind') == 'r27_offline_source_review' and wrapper_review.get('reviewed') is True
        and wrapper_review['baseline_release'] == old_wrapper_review['baseline_release']
        and wrapper_review['source_sha256'] == wanted,
        'new original-wrapper source review differs')
    baseline = read_ref(wrapper_review['baseline_release'])
    require(wrapper_review['changed_sources'] == h4.source_delta(baseline['source_sha256'], wanted)
        and set(wrapper_review['additional_source_files']) == set(old_wrapper_review['additional_source_files']) | NEW_SOURCES
        and len(wrapper_review['additional_source_files']) == len(set(wrapper_review['additional_source_files']))
        and set(wrapper_review['additional_test_files']) == set(old_wrapper_review['additional_test_files']) | set(TESTS)
        and len(wrapper_review['additional_test_files']) == len(set(wrapper_review['additional_test_files'])),
        'target extension changed original source/scope declarations')
    run = read_ref(targeted_ref)
    summary = base.entry.recorded_case_summary(run)
    require(run['source_sha256'] == wanted and set(run['test_files']) == set(TESTS)
        and run['complete_scope_requested'] is False and run['offline_scope_complete'] is False
        and run['full_regression_passed'] is False and run['changed_sources'] == []
        and set(TESTS) <= set(run['explicit_test_files']), 'exact isolated continuation target required')
    production_run = read_ref(base.release['actual_regression'])
    require(set(run['declared_test_files']) == set(production_run['test_files']) | set(TESTS)
        and len(run['declared_test_files']) == len(set(run['declared_test_files']))
        and run['excluded_files'] == production_run['excluded_files'], 'existing full scope was not retained')
    path = Path(targeted_ref['path']); name = path.parent.name
    require(path == B/'ros_log/integrated_regression'/name/'result.json', 'original target output path required')
    entry_ref = ref(M/'regression_entry_receipts'/(name+'.json')); entry = read_ref(entry_ref)
    require(entry['source_review'] == wrapper_review_ref
        and entry['entry'] == dict(path=str(W/h4.REGRESSION_WRAPPER), sha256=base.sources[h4.REGRESSION_WRAPPER])
        and entry['original_runner'] == dict(path=str(W/h4.ORIGINAL_RUNNER), sha256=base.sources[h4.ORIGINAL_RUNNER])
        and entry['storage_binding'] == base.binding
        and entry['manifest_extensions'] == wrapper_review['additional_source_files']
        and entry['scope_extensions'] == wrapper_review['additional_test_files']
        and entry['source_count'] == len(wanted)
        and entry['declared_test_file_count'] == len(run['declared_test_files'])
        and entry['all_original_test_files_preserved'] is True
        and entry['sandbox_and_reconciliation_unchanged'] is True
        and entry['hardware_or_network_operations_authorized'] is False, 'actual target entry/provenance differs')
    argv = entry['argv']
    require(argv.count('--name') == 1 and argv[argv.index('--name')+1] == name
        and argv.count('--storage-backend') == 1 and argv[argv.index('--storage-backend')+1] == 'ssd'
        and argv.count('--pilot-file') == len(TESTS)
        and {argv[i+1] for i,v in enumerate(argv) if v == '--pilot-file'} == set(TESTS),
        'target must actually select both new test files')
    return dict(targeted_regression=targeted_ref, target_summary=summary,
        regression_source_review=wrapper_review_ref, regression_entry_receipt=entry_ref,
        unchanged_production_source_count=len(base.sources), additive_source_count=len(new),
        production_full_regression=base.release['actual_regression'], production_full_repeated=False)


class LoadedRuntime:
    """Actual immutable H4 construction, also exercised by archive integration tests."""
    def __init__(self):
        self.h4 = load_module(H4_PATH, H4_SHA)
        self.base = self.h4.Revision(ANCHORS['loaded_revision'])
        require(self.base.sources[H4_PATH] == H4_SHA, 'H4 runtime source differs')
        self.documents = {key: read_ref(value) for key, value in ANCHORS.items()}
        self.chain = validate_anchor_chain(self.base, self.h4, self.documents)


def validate_new_instruction(continuation, acceptance_ref, reference, instruction):
    base, documents = continuation.base, continuation.documents
    base.entry.validate_instruction(instruction)
    previous = documents['prior_authorization']['user_instruction']
    require(reference['path'] != previous['path'] and reference['sha256'] != previous['sha256'],
            'the consumed prior single-attempt instruction cannot be reused')
    require(instruction.get('maximum_attempts') == 1 and instruction.get('automatic_retry') is False,
            'one separately recorded attempt instruction required')
    require(instruction.get('runtime_revision') == base.reference
        and instruction.get('post_trajectory_continuation') == continuation.reference
        and instruction.get('current_acceptance') == acceptance_ref
        and instruction.get('latest_resume') == base.data['instruction'],
        'new single-attempt ticket must bind this successful continuation and actual resume')


class Continuation:
    def __init__(self, reference):
        self.reference, self.data = reference, read_ref(reference)
        d = self.data
        require(d.get('kind') == 'r27_post_trajectory_continuation_r1' and d.get('anchors') == ANCHORS,
                'exact continuation specification required')
        loaded = LoadedRuntime()
        self.h4, self.base, self.documents, self.chain = loaded.h4, loaded.base, loaded.documents, loaded.chain
        self.review_ref = d['source_review']; self.review = read_ref(self.review_ref)
        self.target = validate_targeted_extension(self.base, self.h4, self.review, d['targeted_regression'])
        require(self.target == d['target_validation'], 'target admission changed')
        self.readiness = load_module(READINESS, self.review['new_source_sha256'][READINESS])
        self.history = self.readiness.validate_transport_history(self.documents['recorded_history'], self.documents['recorded_actions'])
        require(asdict(self.history) == d['transport_history'], 'historical action transport proof changed')
        self.snapshot_diagnostic = self.readiness.validate_recorded_snapshot(self.documents['snapshot'], self.history,
            startup=self.base.typed, assets=self.base.assets)
        self.output = Path(d['acceptance_output'])
        require(self.output.parent == self.base.output
            and re.fullmatch('post_trajectory_acceptance_[a-z0-9_]+', self.output.name), 'finite new acceptance output required')
        self.verify_files()

    def verify_files(self, manual_return=False):
        self.base.verify_files(manual_return=manual_return)
        require(read_ref(self.reference) == self.data and read_ref(self.review_ref) == self.review,
                'continuation input changed')
        for key, reference in ANCHORS.items():
            require(read_ref(reference) == self.documents[key], 'continuation anchor changed: '+key)
        for reference in (self.data['targeted_regression'], self.target['regression_source_review'], self.target['regression_entry_receipt']):
            read_ref(reference)
        require(all(sha(W/path) == digest for path,digest in self.review['new_source_sha256'].items()),
                'continuation source changed')
        self.base.storage.validate(self.output, expected=self.base.binding)

    def save(self, path, value):
        path = Path(path); self.base.storage.validate(path, expected=self.base.binding)
        raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
        with path.open('xb') as stream:
            stream.write(raw); stream.flush(); os.fsync(stream.fileno())
        require(path.read_bytes() == raw, 'continuation evidence readback differs')
        return ref(path)


def prepare(args):
    loaded = LoadedRuntime(); base, h4 = loaded.base, loaded.h4
    review_ref = dict(path=str(args.source_review.resolve()), sha256=args.source_review_sha256)
    targeted_ref = dict(path=str(args.targeted_regression.resolve()), sha256=args.targeted_regression_sha256)
    review = read_ref(review_ref); target = validate_targeted_extension(base, h4, review, targeted_ref)
    readiness = load_module(READINESS, review['new_source_sha256'][READINESS])
    history = readiness.validate_transport_history(loaded.documents['recorded_history'], loaded.documents['recorded_actions'])
    diagnostic = readiness.validate_recorded_snapshot(loaded.documents['snapshot'], history, startup=base.typed, assets=base.assets)
    output = args.output.resolve(); base.storage.validate(output, subtree=base.storage.maintenance_root, expected=base.binding)
    require(not output.exists(), 'new preparation directory required')
    require(re.fullmatch('post_trajectory_acceptance_[a-z0-9_]+', args.runtime_child), 'finite new acceptance name required')
    acceptance_output = base.output/args.runtime_child
    require(not acceptance_output.exists(), 'new acceptance destination required')
    output.mkdir()
    data = dict(schema=1, kind='r27_post_trajectory_continuation_r1', anchors=ANCHORS,
        source_review=review_ref, targeted_regression=targeted_ref, target_validation=target,
        transport_history=asdict(history), historical_snapshot_diagnostic=diagnostic,
        acceptance_output=str(acceptance_output), prepared_ns=time.time_ns(), **NO_ACTION)
    # Use the same exclusive, flushed writer without changing any old receipt.
    temporary = object.__new__(Continuation); temporary.base = base
    reference = temporary.save(output/'continuation.json', data)
    Continuation(reference)
    print(json.dumps(dict(prepared=True, continuation=reference, runtime_reloaded=False), ensure_ascii=False))


def load_live(args):
    require(not sys.flags.optimize and sys.flags.no_user_site, 'unoptimized Python with PYTHONNOUSERSITE=1 required')
    continuation = Continuation(dict(path=str(args.continuation.resolve()), sha256=args.continuation_sha256))
    review_ref = dict(path=str(args.execution_review.resolve()), sha256=args.execution_review_sha256)
    review = read_ref(review_ref)
    require(review.get('kind') == 'r27_post_trajectory_execution_review_r1' and review.get('reviewed') is True
        and review['continuation'] == continuation.reference and review['entry'] == ref(__file__)
        and review['loaded_reload'] == ANCHORS['loaded_reload']
        and review.get('runtime_reload_authorized') is False and review.get('typed_mutations_authorized') is False,
        'reviewed exact accept-only continuation required')
    return continuation, review_ref


def validate_no_held_task(message):
    require(message._connection_header.get('callerid') == '/grasp_task_node'
        and message._connection_header.get('latching') == '1' and message.active is False,
        'current real inactive task required')
    require(message.state == 'IDLE' and type(message.stage) is int and message.stage == 0
        and message.success is False
        and 'HOLDING' not in message.message and 'GRASP_HOLD_UNVERIFIED' not in message.message,
        'held/successful/unknown terminal state is not automatic retry authority')
    return dict(state=message.state, stage=message.stage, message=message.message, active=False, success=False,
                stamp_ns=message.header.stamp.to_nsec())


def wait_ready(continuation, observer, rospy, epoch, timeout_sec=5.):
    deadline = time.monotonic()+timeout_sec; error = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            return continuation.readiness.observed_ready(observer, epoch, rospy, continuation.base.assets,
                continuation.history, startup=continuation.base.typed)
        except RuntimeError as exc:
            error = str(exc); time.sleep(min(.02, max(0., deadline-time.monotonic())))
    raise RuntimeError('post-trajectory typed readiness unavailable without corrective action: '+str(error))


def accept(args):
    continuation, review_ref = load_live(args); base, h4 = continuation.base, continuation.h4
    deployed = continuation.documents['loaded_reload']
    out = continuation.output; require(not out.exists(), 'new post-trajectory acceptance output required'); out.mkdir()
    os.environ.update(base.storage.environment(out/'observer_ros', session=base.session))
    import rospy
    import rosgraph
    from controller_manager_msgs.srv import ListControllers
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_d_driver.msg import PhysicalReferenceState, PhysicalReferenceCommand
    from alicia_d_driver.srv import SetPhysicalReference, SetPhysicalReferenceRequest
    rospy.init_node('r27_post_trajectory_acceptance', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name()); monitor = base.stack.Monitor(base.config); observer = base.typed.Observer(rospy)
    report = dict(schema=1, kind='r27_post_trajectory_acceptance_r1', passed=False,
        continuation=continuation.reference, revision=base.reference, reload=ANCHORS['loaded_reload'],
        failed_initial_acceptance=ANCHORS['failed_initial_acceptance'], execution_review=review_ref,
        reload_execution_review=deployed['execution_review'], source_manifest=base.data['release'],
        contract=base.parent['contract'], continuation_validation=continuation.target,
        transport_history=asdict(continuation.history), started_ns=time.time_ns(),
        manual_only=True, automatic_grasp_started=False, full_restart=False, runtime_reloaded=False,
        typed_initial_contract_ready=False, **NO_ACTION)
    try:
        continuation.verify_files(); base.live_policy(master)
        remote, service = deployed['nodes'][h4.REMOTE], deployed['remote_service']
        expected_aliases = base.alias_snapshot(deployed['construction'])
        require(expected_aliases == deployed['auxiliary_nodes'], 'loaded auxiliary construction differs')
        nodes, aliases = base.graph(master, rospy.get_name(), remote, service, expected_aliases=expected_aliases)
        require(nodes == deployed['nodes'] and aliases == expected_aliases, 'already loaded H4 instance changed')
        require(h4.remote_graph(master) == deployed['remote_graph'], 'loaded remote graph changed')
        for kind in (PhysicalReferenceState, PhysicalReferenceCommand, SetPhysicalReference):
            require(kind._md5sum == base.assets['type_md5'][kind._type], 'generated type MD5 differs')
            path = Path(sys.modules[kind.__module__].__file__).resolve()
            require(sha(path) == base.assets['current_sha256'].get(str(path)), 'generated binding source differs')
        for key,value in base.assets['required_parameters'].items():
            require(rospy.get_param(key) is value and base.config['parameters'].get(key) is value, 'typed policy changed')
        binaries = base.typed.runtime_binaries(dict(nodes=nodes), base.assets)
        graph_before = base.typed.graph_bindings(master, base.assets, probe=True)
        manual = h4.wait_manual(base, monitor, rospy, deployed['state']['epoch_ns'], deployed['state'])
        task = validate_no_held_task(rospy.wait_for_message('/grasp/state', GraspState, timeout=3.))
        ready = wait_ready(continuation, observer, rospy, manual['epoch_ns'])
        query_states = {}
        for (name,_owner),key in zip(base.typed.SERVICES.items(), ('driver','hardware')):
            request = SetPhysicalReferenceRequest(); request.operation = request.QUERY
            response = base.typed.bounded_query(rospy.ServiceProxy(name, SetPhysicalReference), request)
            require(response.success is True and response.code == 'LEGACY', 'post-trajectory QUERY is not empty LEGACY')
            query_states[key] = response.state
        query_wait = {}
        queried = continuation.readiness.wait_for_fixed_query_sources(observer, manual['epoch_ns'], rospy,
            base.assets, continuation.history, query_states, query_wait, startup=base.typed)
        controllers = {c.name:c.state for c in rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)().controller}
        require(all(controllers.get(n) == 'running' for n in ('alicia_controller','hand_controller')), 'controller not running')
        gui = base.stack.gui_windows(nodes['/alicia_supervisor_gui']['pid'])
        simulation = base.stack.load(base.stack.SIM); simulation.UNIT = base.startup['components']['simulation']['unit']
        simulation.wait_ready(timeout=40.)
        require(simulation.service_snapshot() == base.parent['simulation'], 'paired simulation readiness/instance differs')
        binding = base.health.load_binding(Path(deployed['remote_startup_binding']['path']), deployed['remote_startup_binding']['sha256'])
        health = base.health.evaluate(binding, out); require(health['passed'] is True, 'REMOTE_DEPENDENCY_NOT_READY')
        health_ref = continuation.save(out/'remote_readiness.json', health)
        continuation.verify_files(); base.live_policy(master)
        require(base.graph(master, rospy.get_name(), remote, service, expected_aliases=expected_aliases) == (nodes,aliases), 'runtime changed during post-trajectory acceptance')
        final = h4.wait_manual(base, monitor, rospy, manual['epoch_ns'], manual)
        final_task = validate_no_held_task(rospy.wait_for_message('/grasp/state', GraspState, timeout=3.))
        final_typed = continuation.readiness.observed_ready(observer, final['epoch_ns'], rospy, base.assets,
            continuation.history, startup=base.typed)
        graph_after = base.typed.graph_bindings(master, base.assets)
        require(graph_after['topics'] == graph_before['topics'] and all(graph_after['services'][n]['uri'] == v['uri'] for n,v in graph_before['services'].items()), 'typed ownership changed')
        require(base.typed.runtime_binaries(dict(nodes=nodes), base.assets) == binaries, 'binary mapping changed')
        require(0 <= time.time_ns()-health['finished_ns'] < 30_000_000_000, 'remote health expired')
        report.update(passed=True, nodes=nodes, auxiliary_nodes=aliases, remote_service=service, state=final,
            core_unit=base.parent['core_unit'], core_service=base.parent['core_service'],
            master_identity=base.parent['master_identity'], simulation=base.parent['simulation'],
            parent_acceptance=deployed['parent_acceptance'], parent_shutdown=deployed['parent_shutdown'],
            remote_startup_binding=deployed['remote_startup_binding'], remote_dependency=health_ref,
            source_startup_receipt=deployed['source_startup_receipt'], construction=deployed['construction'],
            all_configured_nodes_started=True, source_cache_process_binding=True, memory_attestation=False,
            typed_post_trajectory_contract_ready=True, physical_or_geometric_execution_ready=False,
            observed_post_trajectory_contract=ready, post_query_contract=queried, query_receipt_source_wait=query_wait,
            final_post_trajectory_contract=final_typed, typed_graph_before=graph_before, typed_graph_after=graph_after,
            binaries=binaries, controllers=controllers, gui_windows=gui, retained_node_count=17,
            initial_task=task, final_task=final_task,
            verification=base.stack.current_release_evidence(base.release))
    except BaseException as exc:
        report['error'] = repr(exc); raise
    finally:
        observer.close(); monitor.close(); report['finished_ns'] = time.time_ns()
        continuation.save(out/'result.json', report)
    print(json.dumps(dict(passed=True, result=ref(out/'result.json')), ensure_ascii=False))


def validate_acceptance(continuation, reference, accepted, review_ref):
    base, deployed = continuation.base, continuation.documents['loaded_reload']
    require(reference['path'] == str(continuation.output/'result.json')
        and accepted.get('kind') == 'r27_post_trajectory_acceptance_r1' and accepted.get('passed') is True
        and accepted.get('typed_post_trajectory_contract_ready') is True
        and accepted.get('typed_initial_contract_ready') is False
        and accepted['continuation'] == continuation.reference and accepted['revision'] == base.reference
        and accepted['reload'] == ANCHORS['loaded_reload'] and accepted['execution_review'] == review_ref
        and accepted['reload_execution_review'] == deployed['execution_review']
        and accepted['source_manifest'] == base.data['release'] and accepted['contract'] == base.parent['contract']
        and accepted['transport_history'] == asdict(continuation.history)
        and accepted['continuation_validation'] == continuation.target, 'exact successful post-trajectory acceptance required')
    require(all(accepted[key] == deployed[key] for key in ('nodes','auxiliary_nodes','remote_service',
        'remote_startup_binding','source_startup_receipt','construction','parent_acceptance','parent_shutdown'))
        and accepted['state']['epoch_ns'] == deployed['state']['epoch_ns'], 'acceptance lost the already loaded runtime')
    base.entry.validate_acceptance(accepted)


class AttemptContext:
    """New acceptance chain; unchanged H4 graph/cleanup and original Session."""
    def __init__(self, continuation, acceptance_ref, instruction_ref, execution_review_ref, recorder):
        self.continuation, self.rev = continuation, continuation.base
        self.acceptance_ref, self.instruction_ref = acceptance_ref, instruction_ref
        self.acceptance, self.instruction = read_ref(acceptance_ref), read_ref(instruction_ref)
        self.recorder = recorder; self.typed_observer = None; self.typed_admission = []
        validate_new_instruction(continuation, acceptance_ref, instruction_ref, self.instruction)
        validate_acceptance(continuation, acceptance_ref, self.acceptance, execution_review_ref)
        self.contract, self.config, self.stack = self.rev.contract, self.rev.config, self.rev.stack
        self.release_ref, self.release, self.revision_ref = self.rev.data['release'], self.rev.release, None
        for attribute,path in (('boundary','tools/grasp_attempt_boundary.py'),('finish','tools/finish_returned_grasp.py'),
                ('idle','.runtime/restore_gen62_after_handoff_20261022.py'),('health','.runtime/verify_active_remote_readiness_20261020.py'),
                ('warm','.runtime/prewarm_mode_grasp_runner_20261012.py')):
            setattr(self, attribute, continuation.h4.load_source(path, self.rev.sources))
        self.remote_ref = self.acceptance['remote_startup_binding']
        self.remote = self.health.load_binding(Path(self.remote_ref['path']), self.remote_ref['sha256'])
        require(self.remote['node_identity'] == {k:v for k,v in self.acceptance['nodes'][continuation.h4.REMOTE].items() if k != 'uri'}
            and self.remote['parameters'] == self.config['remote_parameters'], 'accepted remote binding differs')

    def validate_contract(self, manual_return=False):
        self.continuation.verify_files(manual_return=manual_return)
        return self.continuation.h4.AttemptContext.validate_contract(self, manual_return=manual_return)

    def recheck(self, master, caller, manual_return=False):
        self.continuation.h4.AttemptContext.recheck(self, master, caller, manual_return=manual_return)
        if manual_return:
            return
        import rospy
        from alicia_flexible_grasp_supervisor.msg import GraspState
        # Session invokes this before the automatic mode request, then once in
        # automatic selection. No historic seq gate survives into cleanup.
        if rospy.get_param('/gui/joint_direct_mode') is True:
            task = validate_no_held_task(rospy.wait_for_message('/grasp/state', GraspState, timeout=3.))
            if self.typed_observer is None:
                self.typed_observer = self.rev.typed.Observer(rospy)
            ready = wait_ready(self.continuation, self.typed_observer, rospy, self.acceptance['state']['epoch_ns'])
            self.typed_admission.append(dict(task=task, typed=ready, observed_ns=time.time_ns()))

    def close(self):
        if self.typed_observer is not None:
            self.typed_observer.close(); self.typed_observer = None


def attempt(args):
    continuation, execution_review = load_live(args); base, h4 = continuation.base, continuation.h4
    record = Path(os.environ['GRASP_RECORD_DIR']).absolute()
    base.storage.validate(record, subtree=base.storage.archive_root, expected=base.binding)
    require(record != OLD_RECORD and record.is_dir() and record.parent == base.storage.archive_root
        and not (record/'authorization.json').exists(), 'new original recorder attempt required')
    instruction_ref = dict(path=str(args.instruction.resolve()), sha256=args.instruction_sha256)
    recorder = h4.RecorderBinding(base, record, instruction_ref)
    # Explicit child argv binding is additional to the original sibling chain.
    require(recorder.processes['attempt']['argv'][:3] == ['/usr/bin/python3','-B',str(W/SELF)]
        and recorder.processes['attempt']['argv'][3] == 'attempt', 'exact continuation attempt entry required')
    base.entry.configure_attempt_environment(record)
    context = AttemptContext(continuation, dict(path=str(args.acceptance.resolve()), sha256=args.acceptance_sha256),
        instruction_ref, execution_review, recorder)
    # One invocation per successful continuation, including if a new ticket is
    # supplied after an earlier failure. Historical tickets cannot bind this
    # newly sealed acceptance; original task cleanup remains independent.
    continuation.save(continuation.output/'attempt_claim.json',
        dict(instruction=instruction_ref, acceptance=context.acceptance_ref, continuation=continuation.reference,
            loaded_revision=base.reference, record=str(record), recorder_ready=recorder.ready_ref,
            recording_manifest=recorder.manifest_ref, pid=os.getpid(), claimed_ns=time.time_ns(), automatic_retry=False))
    session = base.entry.Session(context, record)
    session.report.update(runtime_revision=base.reference, post_trajectory_continuation=continuation.reference,
        runtime_nodes_changed=False, changed_nodes=[], full_restart=False,
        current_full_regression_passed=base.release['full_regression_passed'],
        continuation_validation=continuation.target, preserved_typed_assets=base.preserved_typed_evidence,
        post_trajectory_admission=context.typed_admission)
    try:
        code = base.entry.attempt_lifecycle(session)
    finally:
        context.close(); session.report['recorder_binding'] = recorder.reference
        closed = session.close()
    return code if closed else (code or 1)


def parser():
    p = argparse.ArgumentParser(description=__doc__); sub = p.add_subparsers(dest='action', required=True)
    q = sub.add_parser('prepare')
    for key in ('source-review','targeted-regression'):
        q.add_argument('--'+key, type=Path, required=True); q.add_argument('--'+key+'-sha256', required=True)
    q.add_argument('--output', type=Path, required=True); q.add_argument('--runtime-child', required=True)
    for action in ('accept','attempt'):
        q = sub.add_parser(action)
        for key in (('continuation','execution-review') if action == 'accept' else
                    ('continuation','execution-review','acceptance','instruction')):
            q.add_argument('--'+key, type=Path, required=True); q.add_argument('--'+key+'-sha256', required=True)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    return {'prepare':prepare,'accept':accept,'attempt':attempt}[args.action](args)


if __name__ == '__main__':
    sys.exit(main())
