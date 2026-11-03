#!/usr/bin/env python3
"""One recorded attempt bound to the current full-stack acceptance.

Invoke only after the user's alignment/continue instruction, inside the existing
recording wrapper. This entry never starts/restarts nodes or repairs execution
confirmation. A terminal stationary task can return to manual while remaining
UNCONFIRMED. The recorder seals the record after this entry returns.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import types
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'tools'))
from experiment_storage import select as select_storage
LAUNCHER_REVISION_PATHS = frozenset((
    'tools/run_recorded_manual_grasp.py',
    'tools/tests/test_run_recorded_manual_grasp.py'))
BOOTSTRAP_SOURCE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/actuation_bootstrap.py'
BOOTSTRAP_GUARD_SOURCE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py'
PREWARM_RELEASE_TIMEOUT_SEC = 30.
MODE_ACK_TIMEOUT_SEC = 5.
SELECTION_TIMEOUT_SEC = 8.
RELEASE_RESERVE_SEC = 1.
REQUIRED_SOURCES = (
    'tools/run_recorded_manual_grasp.py', 'tools/full_stack_start.py',
    'tools/experiment_storage.py',
    'tools/grasp_attempt_boundary.py', 'tools/finish_returned_grasp.py',
    '.runtime/restore_gen62_after_handoff_20261022.py',
    '.runtime/verify_active_remote_readiness_20261020.py',
    '.runtime/prewarm_mode_grasp_runner_20261012.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py',
    BOOTSTRAP_SOURCE, BOOTSTRAP_GUARD_SOURCE)
sys.dont_write_bytecode = True


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def read_ref(reference):
    raw = Path(reference['path']).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == reference['sha256'], 'bound input changed: '+reference['path'])
    return json.loads(raw)


def load_source(relative, sources):
    path = W/relative; raw = path.read_bytes()
    require(relative in sources and hashlib.sha256(raw).hexdigest() == sources[relative],
            'required attempt source unbound/changed: '+relative)
    module = types.ModuleType('_recorded_'+path.stem); module.__file__ = str(path)
    exec(compile(raw, str(path), 'exec'), module.__dict__)
    return module


def validate_instruction(value):
    require(value.get('source') == 'actual_user_message' and bool(value.get('text'))
            and value.get('automatic_grasp_authorized') is True
            and value.get('user_alignment_ready') is True
            and value.get('new_scene_confirmation_required') is False
            and value.get('new_scene_confirmation_claimed') is False,
            'actual post-alignment user instruction required; manual startup is not grasp authority')


def validate_acceptance(value):
    require(value.get('passed') is True and value.get('manual_only') is True
            and value.get('all_configured_nodes_started') is True
            and value.get('source_cache_process_binding') is True
            and value.get('automatic_grasp_started') is False,
            'successful current manual full-stack acceptance required')
    require(bool(value.get('nodes')) and bool(value.get('master_identity'))
            and bool(value.get('core_unit')) and bool(value.get('remote_startup_binding')),
            'acceptance lacks current process/dependency bindings')


def recorded_case_summary(run, allow_skips=False):
    """Validate immutable run evidence; this does not relabel its source version."""
    require(run.get('passed') is True and run.get('source_unchanged') is True
            and run.get('external_boundary_attempts') == []
            and run.get('bad_files') == [] and run.get('duplicate_cases') == []
            and run.get('collection_skips') == [], 'revision regression failed or incomplete')
    files, expected, cases = run['test_files'], run['expected_case_ids'], run['case_results']
    require(type(files) is list and files and len(files) == len(set(files))
            and all(type(p) is str and p in run['source_sha256'] for p in files), 'revision test file scope invalid')
    require(type(expected) is list and expected and all(type(c) is str and '::' in c for c in expected)
            and len(expected) == len(set(expected)), 'revision case inventory invalid')
    require({c.split('::', 1)[0] for c in expected} == set(files), 'revision case/file scope differs')
    require(type(cases) is list and all(type(c) is dict and type(c.get('nodeid')) is str for c in cases),
            'revision case result inventory invalid')
    identities = [c['nodeid'] for c in cases]
    require(len(identities) == len(set(identities)) and set(identities) == set(expected),
            'revision cases duplicated, omitted or unexpected')
    skipped = 0
    for case in cases:
        phases = tuple(case.get(k) for k in ('setup', 'call', 'teardown'))
        if phases == ('passed', 'passed', 'passed'):
            continue
        require(allow_skips and phases in (('passed', 'skipped', 'passed'), ('skipped', None, 'passed'))
                and type(case.get('skip_reason')) is str and bool(case['skip_reason']),
                'revision case phase failed/incomplete: '+case['nodeid'])
        skipped += 1
    summary = dict(tests=len(cases), passed=len(cases)-skipped, skipped=skipped, failures=0, errors=0)
    require(all(run['summary'].get(k) == v for k, v in summary.items() if k not in ('failures', 'errors'))
            and run['summary'].get('failed') == 0, 'revision run summary differs from case phases')
    return summary


def recorded_baseline_verification(release, run, stack):
    """Preserve the original current-release scope and every verification field."""
    summary = recorded_case_summary(run, allow_skips=True)
    files, roots, explicit = run['test_files'], run['test_roots'], run['explicit_test_files']
    exclusions = run.get('excluded_files', [])
    require(type(exclusions) is list and all(type(row) is dict and type(row.get('path')) is str
            and type(row.get('reason')) is str and row['reason'] and row['path'] not in files for row in exclusions),
            'test exclusions must be explicit and outside executed scope')
    require(len({row['path'] for row in exclusions}) == len(exclusions), 'duplicate test exclusions')
    require(type(roots) is list and roots and len(roots) == len(set(roots))
            and all(type(p) is str and not Path(p).is_absolute() and '..' not in Path(p).parts
                    and (W/p).is_dir() for p in roots), 'current discovery roots invalid')
    require(all(any(Path(required) == Path(root) or Path(root) in Path(required).parents for root in roots)
                for required in stack.CURRENT_TEST_ROOTS), 'current core/tools test roots omitted')
    require(type(explicit) is list and len(explicit) == len(set(explicit))
            and all(type(p) is str and not Path(p).is_absolute() and '..' not in Path(p).parts
                    and (W/p).is_file() for p in explicit), 'explicit current test files invalid')
    if (W/stack.HISTORICAL_PREWARM_TEST).exists():
        require(stack.HISTORICAL_PREWARM_TEST in files, 'historical prewarm coverage omitted')
    discovered = {str(p.relative_to(W)) for root in roots for p in (W/root).rglob('test_*.py')
                  if '__pycache__' not in p.parts} | set(explicit)
    require(discovered == set(files) | {row['path'] for row in exclusions},
            'current discovered test files omitted or invented')
    require(type(run.get('scope_name')) is str and bool(run['scope_name']), 'current regression scope must be named')
    skipped = [dict(nodeid=c['nodeid'], reason=c['skip_reason']) for c in run['case_results']
               if c['setup'] == 'skipped' or c['call'] == 'skipped']
    complete = not skipped
    require(type(release.get('full_regression_passed')) is bool
            and release['full_regression_passed'] is complete and run.get('full_regression_passed') is complete,
            'current full-regression claim differs from executed bodies')
    return dict(current=release['actual_regression'], scope=run['scope_name'], current_summary=summary,
        exact_case_inventory=True, test_file_count=len(files), test_roots=roots, excluded_files=exclusions,
        skipped_cases=skipped, test_bodies_executed=complete, current_full_regression_passed=complete)


def launcher_revision_sources(revision, acceptance_ref, acceptance, contract, release, stack=None):
    """An exact two-file launcher delta; it never changes the accepted node release."""
    require(revision.get('schema') == 1 and revision.get('kind') == 'launcher_only_revision'
            and revision.get('reviewed') is True and revision.get('runtime_nodes_changed') is False
            and revision.get('current_full_regression_passed') is False, 'invalid launcher-only revision')
    require(revision['startup_acceptance'] == acceptance_ref
            and revision['baseline_contract'] == acceptance['contract']
            and revision['baseline_release'] == acceptance['source_manifest'] == contract['release']
            and revision['baseline_regression'] == release['actual_regression'], 'launcher revision baseline differs')
    require(release.get('kind') == 'current_offline_release' and release.get('schema') == 1
            and all(release.get(k) is True for k in ('passed', 'reviewed', 'installed'))
            and release.get('source_count') == len(release['source_sha256']), 'accepted current release required')
    require(acceptance['verification'] == contract['verification']
            and contract['verification']['current'] == release['actual_regression'], 'accepted verification differs')
    baseline = read_ref(release['actual_regression'])
    require(baseline['source_sha256'] == release['source_sha256']
            and baseline.get('offline_scope_complete') is True, 'accepted regression source/scope differs')
    if stack is None:
        stack = load_source('tools/full_stack_start.py', release['source_sha256'])
    require(recorded_baseline_verification(release, baseline, stack) == contract['verification'],
            'accepted regression verification differs')
    changed = revision['changed_sources']
    require(type(changed) is dict and set(changed) == LAUNCHER_REVISION_PATHS,
            'launcher revision may replace only the entry and its test')
    sources = dict(release['source_sha256'])
    for name, row in changed.items():
        require(set(row) == {'before_sha256', 'after_sha256'} and row['before_sha256'] == sources[name]
                and row['after_sha256'] != row['before_sha256']
                and type(row['after_sha256']) is str and len(row['after_sha256']) == 64
                and all(c in '0123456789abcdef' for c in row['after_sha256']), 'launcher revision source delta invalid')
        sources[name] = row['after_sha256']
    targeted = read_ref(revision['targeted_regression'])
    require(targeted['source_sha256'] == sources and targeted.get('full_regression_passed') is False
            and targeted.get('complete_scope_requested') is False
            and 'tools/tests/test_run_recorded_manual_grasp.py' in targeted['test_files'],
            'launcher targeted source/scope differs')
    recorded_case_summary(targeted)
    return sources


def validate_automatic_entry_snapshot(samples, states, now_ns, epoch_ns, owners, policy, boundary):
    """Share strict idle telemetry, then apply the task's distinct actuation routes."""
    from alicia_flexible_grasp.robot.actuation_bootstrap import PENDING_BOOTSTRAP_STATES
    evidence = boundary.validate_manual_return_snapshot(samples, states, now_ns, epoch_ns, owners)
    require(evidence['feedback_ready'] is True, 'automatic entry requires ready accepted SDK feedback')
    status, enabled = evidence['actuation_status'], evidence['motion_enabled']
    confirmed = status.startswith('CONFIRMED:')
    if confirmed:
        require(enabled is True, 'confirmed actuation and motion_enabled disagree')
        freshness = policy['/grasp/actuation_confirmation_freshness_sec']
        require(type(freshness) in (int, float) and math.isfinite(freshness) and freshness >= 0.,
                'invalid bound actuation freshness')
        require(0 <= (now_ns-states['actuation'][1])*1e-9 <= freshness, 'actuation confirmation expired')
    else:
        require(status in PENDING_BOOTSTRAP_STATES and enabled is False,
                'automatic entry is neither confirmed nor an eligible initial pending state: '+status)
        require(all(policy.get(key) is True for key in (
            '/grasp/require_actuation_confirmation', '/grasp/auto_confirm_actuation_from_observation',
            '/grasp/use_grasp6d_plan')), 'bound observation actuation bootstrap is not configured')
        require(0 <= now_ns-states['actuation'][1] <= 2_000_000_000, 'pending actuation status expired')
    evidence.update(purpose='automatic_entry_idle', automatic_actuation_gate_passed=confirmed,
        actuation_confirmed=confirmed,
        entry_actuation_state='confirmed' if confirmed else 'pending_observation_bootstrap_eligible',
        pending_observation_bootstrap_eligible=not confirmed,
        confirmation_required_before_full_task_motion=not confirmed)
    # Eligibility only releases the existing plan-bound task bootstrap. It does
    # not confirm the driver, authorize an arbitrary probe, or bypass its gates.
    return evidence


def bound_parameter_tree(parameters):
    """Rebuild roslaunch's flat names, rejecting ambiguous parent/leaf bindings."""
    require(type(parameters) is dict, 'accepted parameter mapping invalid')
    tree, assigned = {}, set()
    def component(value):
        return (type(value) is str and bool(value) and value not in ('.', '..')
                and all(c.isalnum() or c == '_' for c in value))
    def valid_value(value):
        if type(value) is dict:
            return all(component(k) and valid_value(v) for k, v in value.items())
        if type(value) is list:
            return all(valid_value(v) for v in value)
        return type(value) in (str, bool, int) or (type(value) is float and math.isfinite(value))
    def insert(parts, value):
        require(valid_value(value), 'accepted parameter value invalid: /'+'/'.join(parts))
        if type(value) is dict and value:
            for name, child in value.items():
                insert(parts+(name,), child)
            return
        cursor = tree
        for index, part in enumerate(parts[:-1], 1):
            require(parts[:index] not in assigned and type(cursor.get(part, {})) is dict,
                    'accepted parameter parent/leaf conflict: /'+'/'.join(parts))
            cursor = cursor.setdefault(part, {})
        require(parts[-1] not in cursor, 'accepted parameter duplicate/parent conflict: /'+'/'.join(parts))
        cursor[parts[-1]] = value
        assigned.add(parts)
    for name, value in parameters.items():
        require(type(name) is str and name.startswith('/'), 'accepted parameter name is not absolute')
        parts = tuple(name[1:].split('/'))
        require(all(component(p) for p in parts), 'accepted parameter name invalid: '+name)
        insert(parts, value)
    return tree


def parameter_values_equal(actual, expected):
    """ROS bool/int and int/float differences remain significant at every depth."""
    if type(actual) is not type(expected):
        return False
    if type(expected) is dict:
        return (all(type(k) is str for k in actual) and set(actual) == set(expected)
                and all(parameter_values_equal(actual[k], value) for k, value in expected.items()))
    if type(expected) is list:
        return len(actual) == len(expected) and all(parameter_values_equal(a, b) for a, b in zip(actual, expected))
    return actual == expected


def read_bound_parameters(parameters, get_param, keys):
    tree, values = bound_parameter_tree(parameters), {}
    for name in keys:
        require(type(name) is str and name.startswith('/'), 'requested policy name is not absolute')
        expected = tree
        for part in name.lstrip('/').split('/'):
            require(type(expected) is dict and part in expected, 'accepted parameter missing: '+name)
            expected = expected[part]
        try:
            actual = get_param(name)
        except KeyError as exc:
            raise RuntimeError('runtime parameter missing: '+name) from exc
        require(parameter_values_equal(actual, expected), 'accepted runtime policy changed: '+name)
        values[name] = actual
    return values


def configure_attempt_environment(record):
    cache = record/'unused_python_cache'; require(not cache.exists(), 'attempt cache prefix already populated')
    os.environ.update(ROS_LOG_DIR=str(record/'runner_ros'),PYTHONPYCACHEPREFIX=str(cache),PYTHONDONTWRITEBYTECODE='1')
    # The environment configures the prewarm child; this interpreter is already
    # running and must also stop reading existing package-local bytecode.
    sys.pycache_prefix = str(cache)


def attempt_lifecycle(adapter):
    """Preserve the task result; a failed cleanup is a separate launcher failure."""
    code, primary, cleaned = None, None, False
    try:
        adapter.preflight()
        adapter.enter_automatic()
        selected = adapter.select()
        code = adapter.run_selected(selected)
        require(type(code) is int, 'original runner did not return an exit code')
        adapter.report['runner_returncode'] = code
    except BaseException as exc:
        adapter.report['launcher_error'] = repr(exc)
        primary = sys.exc_info()
    finally:
        try:
            adapter.report['manual_return'] = adapter.return_manual()
            cleaned = True
        except BaseException as exc:
            adapter.report['manual_return_error'] = repr(exc)
        adapter.report.update(finished_ns=time.time_ns(), manual_return_verified=cleaned)
        adapter.save()
    if primary is not None:
        raise primary[1].with_traceback(primary[2])
    return code if cleaned or code else 1


def wait_selection_commit(rospy, prior, timeout_sec=SELECTION_TIMEOUT_SEC):
    """Require the router's own completed selection and status transaction."""
    from std_msgs.msg import String
    from alicia_grasp_modes.selection import parse_selection
    lock, received, subscriptions = threading.Lock(), {}, []
    deadline = time.monotonic()+timeout_sec
    def receive(name, message):
        if (message._connection_header.get('callerid') != '/grasp_mode_router'
                or message._connection_header.get('latching') != '1'):
            return
        try:
            payload = json.loads(message.data); selected = parse_selection(payload)
        except (TypeError, ValueError):
            return
        with lock: received[name] = (selected,payload)
    selected = None
    try:
        for name in ('selection','status'):
            subscriptions.append(rospy.Subscriber('/grasp_mode/'+name,String,
                lambda msg,key=name:receive(key,msg),queue_size=2))
        while time.monotonic() < deadline and not rospy.is_shutdown():
            candidate = parse_selection(rospy.get_param('/grasp_mode/selection'))
            if candidate != prior:
                require(candidate['generation'] == prior['generation']+1 and candidate['stamp_ns'] > prior['stamp_ns']
                        and candidate['mode'] == 'unknown' and candidate['strategy'] == 'observe_return',
                        'selection is not the exact requested successor')
                require(selected is None or selected == candidate, 'selection changed during commit acknowledgement')
                selected = candidate
            else:
                require(selected is None, 'selection reverted during commit acknowledgement')
            with lock: committed,status = received.get('selection'),received.get('status')
            if selected is not None and status is not None and status[0] == selected:
                require(status[1].get('state') != 'error', 'selection reset failed: '+str(status[1].get('detail','')))
                if committed is not None and committed[0] == selected and status[1].get('state') in ('waiting_for_target','ready'):
                    require(parse_selection(rospy.get_param('/grasp_mode/selection')) == selected,
                            'selection changed before runner release')
                    require(time.monotonic() < deadline and not rospy.is_shutdown(), 'selection commit expired')
                    return selected,dict(status[1])
            time.sleep(.025)
        raise RuntimeError('router-own selection commit acknowledgement missing; no retry')
    finally:
        for sub in subscriptions: sub.unregister()


class Context:
    def __init__(self, acceptance_ref, instruction_ref, revision_ref=None):
        self.acceptance_ref, self.instruction_ref = acceptance_ref, instruction_ref
        self.acceptance = read_ref(acceptance_ref); validate_acceptance(self.acceptance)
        self.instruction = read_ref(instruction_ref); validate_instruction(self.instruction)
        self.release_ref = self.acceptance['source_manifest']; self.release = read_ref(self.release_ref)
        require(set(REQUIRED_SOURCES) <= set(self.release['source_sha256']), 'attempt dependencies missing from release')
        sources = self.release['source_sha256']
        self.stack = load_source('tools/full_stack_start.py', sources)
        self.boundary = load_source('tools/grasp_attempt_boundary.py', sources)
        self.finish = load_source('tools/finish_returned_grasp.py', sources)
        self.idle = load_source('.runtime/restore_gen62_after_handoff_20261022.py', sources)
        self.health = load_source('.runtime/verify_active_remote_readiness_20261020.py', sources)
        self.warm = load_source('.runtime/prewarm_mode_grasp_runner_20261012.py', sources)
        self.contract = read_ref(self.acceptance['contract'])
        require(self.contract['release'] == self.release_ref, 'acceptance/release contract differs')
        self.revision_ref = revision_ref
        self.revision = read_ref(revision_ref) if revision_ref is not None else None
        _, self.config, installed = self.validate_contract()
        require(installed == self.release, 'accepted installed release differs')
        self.remote_ref = self.acceptance['remote_startup_binding']
        self.remote = self.health.load_binding(Path(self.remote_ref['path']), self.remote_ref['sha256'])
        actual_remote = self.acceptance['nodes']['/remote_grasp6d_node']
        require(self.remote['node_identity'] == {k:actual_remote[k] for k in ('pid','parent_pid','start_ticks','cmdline_sha256')}
                and self.remote['parameters'] == self.config['remote_parameters'], 'accepted remote construction binding differs')

    def validate_contract(self, *, manual_return=False):
        if not manual_return:
            require(read_ref(self.contract['control']).get('state') == 'active',
                    'automatic grasp requires active control')
        if self.revision is None:
            return self.stack.validate_contract(self.contract, manual_return=manual_return)
        require(read_ref(self.revision_ref) == self.revision
                and read_ref(self.acceptance['contract']) == self.contract
                and read_ref(self.release_ref) == self.release, 'launcher revision baseline inputs changed')
        sources = launcher_revision_sources(self.revision, self.acceptance_ref, self.acceptance,
            self.contract, self.release, self.stack)
        # Same startup invariants as full_stack_start.validate_contract, with
        # only the separately tested two-file source overlay above. Never pass
        # an altered release into that validator or change its implementation.
        contract = self.contract
        require(contract['schema'] == 1 and contract['kind'] == 'manual_full_stack_cold_start'
                and contract['workspace_root'] == str(W), 'wrong startup contract')
        out = Path(contract['output_root'])
        self.stack.storage(out, manual_return=manual_return, backend=contract['storage']['backend'],
                           expected=contract['storage'])
        for path, digest in contract['current_sha256'].items():
            revised = next((name for name in LAUNCHER_REVISION_PATHS if W/name == Path(path)), None)
            if revised is not None:
                require(digest == self.release['source_sha256'][revised], 'startup revision baseline digest differs')
                digest = sources[revised]
            require(self.stack.sha(path) == digest, 'current startup input changed: '+path)
        instruction, control = read_ref(contract['instruction']), read_ref(contract['control'])
        require(instruction.get('source') == 'actual_user_message' and instruction.get('full_restart_authorized') is True
                and instruction.get('new_scene_confirmation_claimed') is False and bool(instruction.get('text')),
                'actual restart instruction required')
        require(control['state'] == 'active' and control == contract['control_snapshot'], 'current control changed')
        self.stack.source_check(sources)
        config = read_ref(contract['configuration'])
        require(self.stack.configuration(session=out) == config, 'expanded launch/source configuration changed')
        require(not Path(contract['pycache_prefix']).exists(), 'fresh unused Python cache prefix was populated')
        require(self.stack.sha(W/'devel/lib/alicia_d_driver/alicia_d_driver_node') == contract['driver_binary_sha256'],
                'driver binary changed')
        return out, config, self.release

    def recheck(self, master, caller, manual_return=False):
        require(read_ref(self.acceptance_ref) == self.acceptance and read_ref(self.instruction_ref) == self.instruction,
                'attempt inputs changed')
        self.validate_contract(manual_return=manual_return)
        require(self.stack.properties(self.acceptance['core_unit']) == self.acceptance['core_service'],
                'accepted core service changed')
        require(self.stack.process(master.getPid()) == self.acceptance['master_identity'], 'accepted ROS master changed')
        if not manual_return:
            self.health.check_binding(self.remote, self.health.observe(self.remote))
        required = (set(self.idle.OWNERS.values()) | {'/motion_gateway'}) if manual_return else set(self.acceptance['nodes'])
        require(required <= set(self.acceptance['nodes']), 'accepted control owners missing')
        for name in sorted(required):
            expected = self.acceptance['nodes'][name]
            uri = master.lookupNode(name)
            reply = xmlrpc.client.ServerProxy(uri, transport=self.health.RpcTransport()).getPid(caller)
            require(reply[0] == 1, 'accepted node unavailable: '+name)
            row = self.stack.process(reply[2])
            raw = (Path('/proc')/str(row['pid'])/'cmdline').read_bytes()
            observed = dict(row, cmdline_sha256=hashlib.sha256(raw).hexdigest(), uri=uri)
            require(observed == expected, 'accepted node identity changed: '+name)


class Session:
    def __init__(self, context, record):
        self.ctx, self.record = context, record
        self.lock, self.states, self.samples = threading.RLock(), {}, []
        self.subscriptions, self.publisher, self.prepared = [], None, None
        self.attempt_epoch, self.finish_epoch = None, None
        self.selected, self.automatic_requested = None, False
        self.prewarm_deadline = None
        self.report = dict(schema=1, kind='current_recorded_attempt', started_ns=time.time_ns(),
            startup_acceptance=context.acceptance_ref, source_manifest=context.release_ref,
            user_instruction=context.instruction_ref, record_dir=str(record), automatic_retry=False,
            physical_grasp_verified=False, scene_review_skipped_by_user=True,
            new_scene_confirmation_claimed=False, temperature_queried=False, driver_restarted=False)
        if context.revision_ref is not None:
            self.report.update(launcher_revision=context.revision_ref,
                current_full_regression_passed=False, runtime_nodes_changed=False)

    def save(self):
        selected = select_storage(path=self.record)
        selected.validate(self.record, subtree=selected.archive_root, expected=self.ctx.contract['storage'])
        raw = (json.dumps(self.report, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
        path = self.record/'authorization.json'; temporary = self.record/'authorization.pending.json'
        with temporary.open('wb') as stream:
            stream.write(raw); stream.flush(); os.fsync(stream.fileno())
        require(temporary.read_bytes() == raw, 'attempt receipt readback differs')
        os.replace(temporary, path)

    def connect(self):
        import rospy
        import rosgraph
        from std_msgs.msg import Bool, Header, String
        from sensor_msgs.msg import JointState
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.rospy, self.Bool = rospy, Bool
        rospy.init_node('current_recorded_grasp', anonymous=True, disable_rosout=True)
        self.master = rosgraph.Master(rospy.get_name())
        self.topics = [('joint','/alicia_d/accepted_joint_states',JointState),
            ('epoch','/alicia_d/control_reference_epoch',Header), ('manual','/gui/joint_direct_mode',Bool),
            ('task','/grasp/state',GraspState), ('mode','/grasp_mode/status',String),
            ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
            ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
            ('actuation','/alicia_d/actuation_status',String), ('ready','/alicia_d/feedback_ready',Bool),
            ('enabled','/alicia_d/motion_enabled',Bool)]
        self.subscriptions = [rospy.Subscriber(topic, kind, lambda msg,k=key:self.receive(k,msg), queue_size=40)
            for key,topic,kind in self.topics]

    def receive(self, key, message):
        if message._connection_header.get('callerid') != self.ctx.idle.OWNERS[key]:
            return
        now = self.rospy.Time.now().to_nsec()
        with self.lock:
            self.states[key] = (message, now)
            if key == 'joint':
                self.samples.append(dict(stamp_ns=message.header.stamp.to_nsec(), received_ns=now,
                    names=list(message.name), positions=list(message.position), frame_id=message.header.frame_id,
                    caller=message._connection_header['callerid']))
                self.samples[:] = self.samples[-40:]

    def policy(self, manual_return=False):
        keys = ('/alicia_d_driver_node/software_temperature_monitoring_enabled',
            '/alicia_d_driver_node/temperature_poll_rate_hz','/alicia_d_driver_node/self_check_poll_rate_hz')
        if not manual_return:
            keys += ('/camera/simulate','/camera/fallback_to_simulation','/camera/mode_depth_preset',
                '/grasp/unknown_lift_after_close_enabled','/grasp/unknown_observe_return_max_observations',
                '/grasp/require_actuation_confirmation','/grasp/auto_confirm_actuation_from_observation',
                '/grasp/use_grasp6d_plan','/grasp/actuation_confirmation_freshness_sec')
        return read_bound_parameters(self.ctx.config['parameters'], self.rospy.get_param, keys)

    def observe(self, manual_return=False, timeout_sec=5.):
        self.ctx.recheck(self.master, self.rospy.get_name(), manual_return=manual_return)
        policy = self.policy(manual_return=manual_return)
        deadline, error, first_error = time.monotonic()+timeout_sec, 'SDK/idle telemetry pending', None
        if not manual_return:
            # A first import can take longer than the SDK freshness limit.
            # Load it before taking any snapshot or its validation timestamp.
            from alicia_flexible_grasp.robot.actuation_bootstrap import PENDING_BOOTSTRAP_STATES
        while time.monotonic() < deadline and not self.rospy.is_shutdown():
            with self.lock:
                window_ready = len(self.samples) >= 20
            if not window_ready:
                error = 'SDK stationary sample window incomplete'
                time.sleep(min(.025, max(0., deadline-time.monotonic())))
                continue
            try:
                # At 10 Hz, a new 20-frame window takes about 1.9 s. Read the
                # event-latched states after that wait, and again on a retry.
                # A latch replay is a snapshot, not a new network heartbeat;
                # the unchanged guard still requires live SDK/action feedback.
                for key,topic,kind in self.topics:
                    if key not in ('joint','arm','hand'):
                        remaining = deadline-time.monotonic()
                        require(remaining > 0., 'snapshot observation deadline exhausted')
                        self.receive(key, self.rospy.wait_for_message(topic,kind,timeout=min(3.,remaining)))
                require(time.monotonic() < deadline and not self.rospy.is_shutdown(),
                        'snapshot observation deadline exhausted')
                with self.lock:
                    samples, states = list(self.samples[-20:]), dict(self.states)
                epoch = states['epoch'][0].stamp.to_nsec()
                require(epoch >= self.ctx.acceptance['state']['epoch_ns'], 'driver epoch predates accepted startup')
                if manual_return and self.attempt_epoch is not None:
                    require(epoch >= self.attempt_epoch, 'terminal epoch predates attempt')
                fixed = self.finish_epoch if manual_return else self.attempt_epoch
                require(fixed is None or epoch == fixed, 'attempt/finish reference epoch changed')
                now_ns = self.rospy.Time.now().to_nsec()
                if manual_return:
                    evidence = self.ctx.boundary.validate_manual_return_snapshot(samples, states,
                        now_ns, epoch, self.ctx.idle.OWNERS)
                else:
                    evidence = validate_automatic_entry_snapshot(samples, states, now_ns, epoch,
                        self.ctx.idle.OWNERS, policy, self.ctx.boundary)
                publishers = dict(self.master.getSystemState()[0])
                for key,topic,_ in self.topics:
                    owners = [self.ctx.idle.OWNERS[key]]
                    if key == 'manual' and self.publisher is not None:
                        owners.append(self.rospy.get_name())
                    require(sorted(publishers.get(topic,[])) == sorted(owners), 'topic publisher ownership differs: '+topic)
                from alicia_grasp_modes.selection import parse_selection
                selected = parse_selection(self.rospy.get_param('/grasp_mode/selection'))
                require(self.selected is None or selected == self.selected, 'attempt selection changed')
                parameter = self.rospy.get_param('/gui/joint_direct_mode')
                require(type(parameter) is bool and states['manual'][0].data is parameter, 'GUI-own mode acknowledgement differs')
                if manual_return: self.finish_epoch = epoch
                else: self.attempt_epoch = epoch
                return dict(manual_parameter=parameter,gui_manual_ack=states['manual'][0].data,
                    gui_ack_received_ns=states['manual'][1],epoch_ns=epoch,node_identities=self.ctx.acceptance['nodes'],
                    selection=selected,policy=policy,latest_sdk_stamp_ns=samples[-1]['stamp_ns'],idle=evidence)
            except (RuntimeError, ValueError, AssertionError, KeyError) as exc:
                error = repr(exc)
                if first_error is None: first_error = error
                time.sleep(min(.025, max(0., deadline-time.monotonic())))
        raise RuntimeError('attempt boundary unavailable: first_error='+str(first_error or error)+'; last_error='+error)

    def prepare_publisher(self):
        if self.publisher is None:
            self.publisher = self.rospy.Publisher('/gui/joint_direct_mode',self.Bool,queue_size=1,latch=True)
        deadline = time.monotonic()+3.
        while not self.publisher.get_num_connections() and time.monotonic() < deadline:
            time.sleep(.025)
        require(self.publisher.get_num_connections() > 0, 'mode request has no subscriber')

    def request_mode(self, value, snapshot):
        requested = self.rospy.Time.now().to_nsec()
        require(0 <= requested-snapshot['latest_sdk_stamp_ns'] < 200_000_000, 'SDK expired before mode request')
        self.rospy.set_param('/gui/joint_direct_mode',value)
        self.publisher.publish(self.Bool(data=value))
        return requested

    def wait_ack(self, value, requested):
        deadline = time.monotonic()+MODE_ACK_TIMEOUT_SEC
        while time.monotonic() < deadline and not self.rospy.is_shutdown():
            with self.lock: ack = self.states.get('manual')
            if (ack and ack[0].data is value and ack[1] >= requested
                    and ack[0]._connection_header.get('callerid') == self.ctx.idle.GUI
                    and ack[0]._connection_header.get('latching') == '1'
                    and self.rospy.get_param('/gui/joint_direct_mode') is value):
                return
            time.sleep(.025)
        raise RuntimeError('fresh GUI-own mode acknowledgement unavailable')

    def preflight(self):
        self.connect()
        remote = self.ctx.health.load_binding(Path(self.ctx.remote_ref['path']),self.ctx.remote_ref['sha256'])
        self.report['remote_readiness'] = self.ctx.health.evaluate(remote,self.record)
        require(self.report['remote_readiness']['passed'] is True, 'REMOTE_DEPENDENCY_NOT_READY')
        self.report['before'] = self.observe()
        require(self.report['before']['manual_parameter'] is True, 'attempt must begin in acknowledged manual mode')
        self.save()

    def require_prewarm_budget(self, reserve_sec):
        require(self.prepared is not None and not self.prepared.released
                and self.prepared.process.poll() is None, 'prewarmed runner exited or was already released')
        remaining = self.prewarm_deadline-time.monotonic()
        require(remaining > reserve_sec, 'prewarmed runner release budget exhausted before next handoff step')
        return remaining

    def enter_automatic(self):
        self.prepare_publisher()
        # Expensive initial source/evidence/health checks ran in preflight. The
        # imports-only child is created before mode transfer, with the original
        # 30 s release timeout. Budget from constructor entry is conservative:
        # the child's own release timer starts later, after its READY write.
        started = time.monotonic()
        self.prepared = self.ctx.warm.PrewarmedModeRunner(
            W/'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py',
            release_timeout=PREWARM_RELEASE_TIMEOUT_SEC)
        self.prewarm_deadline = started+PREWARM_RELEASE_TIMEOUT_SEC
        self.report.update(prewarm=self.prepared.ready, prewarm_ready_ns=time.time_ns(),
            prewarm_import_sec=time.monotonic()-started, prewarm_release_timeout_sec=PREWARM_RELEASE_TIMEOUT_SEC)
        current = self.observe()
        require(current['manual_parameter'] is True, 'manual mode changed before automatic request')
        require(0 <= time.time_ns()-self.report['remote_readiness']['finished_ns'] < 30_000_000_000,
                'remote readiness expired before automatic request')
        remaining = self.require_prewarm_budget(MODE_ACK_TIMEOUT_SEC+SELECTION_TIMEOUT_SEC+RELEASE_RESERVE_SEC)
        self.report.update(automatic_entry=current, prewarm_remaining_before_mode_sec=remaining)
        self.automatic_requested = True
        requested = self.request_mode(False,current); self.wait_ack(False,requested)
        self.report['automatic_ack'] = dict(requested_ns=requested)

    def select(self):
        from alicia_grasp_modes.srv import SetGraspMode
        from alicia_grasp_modes.selection import parse_selection
        self.require_prewarm_budget(SELECTION_TIMEOUT_SEC+RELEASE_RESERVE_SEC)
        self.ctx.recheck(self.master,self.rospy.get_name())
        services = dict(self.master.getSystemState()[2])
        require(all(services.get(name) == ['/grasp_mode_router'] for name in ('/grasp_mode/select','/grasp/start')),
                'grasp service ownership changed')
        prior = parse_selection(self.rospy.get_param('/grasp_mode/selection'))
        self.require_prewarm_budget(SELECTION_TIMEOUT_SEC+RELEASE_RESERVE_SEC)
        response = self.rospy.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown','observe_return')
        require(response.success, response.message)
        self.selected, ack = wait_selection_commit(self.rospy,prior)
        self.report.update(selection=self.selected,selection_ack=ack,selection_committed_ns=time.time_ns())
        self.save()
        return self.selected

    def run_selected(self, selected):
        self.report['prewarm_remaining_before_release_sec'] = self.require_prewarm_budget(RELEASE_RESERVE_SEC)
        return self.prepared.run_once(selected)

    def return_manual(self):
        if self.prepared is not None: self.prepared.close()
        if not hasattr(self,'rospy'):
            return dict(outcome='no_ROS_connection_or_mode_request',manual_request_sent=False)
        session = self
        class Adapter:
            def verify_closed(self):
                require(session.prepared is None or not session.prepared.released or session.prepared.process.poll() is not None,
                        'original runner still owns execution; manual restoration held')
                log = session.record/'runner.log'
                require(not log.exists() or 'execution_response_unknown=True' not in log.read_text(errors='replace'),
                        'ambiguous execution response remains with task owner')
            def recheck(self): session.ctx.recheck(session.master,session.rospy.get_name(),manual_return=True)
            def observe_idle(self): return session.observe(manual_return=True)
            def prepare_manual_request(self): session.prepare_publisher()
            def request_manual(self,snapshot): return session.request_mode(True,snapshot)
            def wait_gui_own_ack(self,requested): session.wait_ack(True,requested)
        result = self.ctx.finish.finish_returned_attempt(Adapter())
        result['automatic_motion_authorized'] = False
        result['recorder_sealed'] = False
        result['terminal_epoch_differs_from_attempt'] = self.attempt_epoch is not None and self.finish_epoch != self.attempt_epoch
        return result

    def close(self):
        errors = []
        for handle in ([self.publisher] if self.publisher is not None else [])+self.subscriptions:
            try: handle.unregister()
            except BaseException as exc: errors.append(repr(exc))
        if hasattr(self,'rospy'):
            try: self.rospy.signal_shutdown('current recorded attempt ended')
            except BaseException as exc: errors.append(repr(exc))
        self.report['transport_cleanup_errors'] = errors
        self.save()
        return not errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('startup-acceptance','instruction'):
        parser.add_argument('--'+key,type=Path,required=True)
        parser.add_argument('--'+key+'-sha256',required=True)
    parser.add_argument('--launcher-revision',type=Path)
    parser.add_argument('--launcher-revision-sha256')
    args = parser.parse_args(argv)
    require(bool(args.launcher_revision) == bool(args.launcher_revision_sha256), 'launcher revision path/SHA required together')
    require(not sys.flags.optimize, 'unoptimized Python required for existing automatic admission checks')
    require(sys.flags.no_user_site, 'run with PYTHONNOUSERSITE=1')
    record = Path(os.environ['GRASP_RECORD_DIR']).absolute()
    selected = select_storage(path=record)
    selected.validate(record, subtree=selected.archive_root)
    require(record.is_dir() and record.parent == selected.archive_root, 'current recording directory required')
    ready = json.loads((record/'recorder_ready.json').read_bytes())
    require(ready['record_dir'] == str(record) and type(ready['pid']) is int
            and (Path('/proc')/str(ready['pid'])).is_dir(), 'current recorder not ready')
    require(not (record/'authorization.json').exists(), 'record already has an attempt authorization')
    configure_attempt_environment(record)
    context = Context(dict(path=str(args.startup_acceptance.resolve()),sha256=args.startup_acceptance_sha256),
        dict(path=str(args.instruction.resolve()),sha256=args.instruction_sha256),
        dict(path=str(args.launcher_revision.resolve()),sha256=args.launcher_revision_sha256)
        if args.launcher_revision is not None else None)
    session = Session(context,record)
    try:
        code = attempt_lifecycle(session)
    finally:
        closed = session.close()
    return code if closed else (code or 1)


if __name__ == '__main__':
    sys.exit(main())
