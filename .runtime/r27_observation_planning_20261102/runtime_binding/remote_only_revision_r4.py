#!/usr/bin/python3
"""Finite R27 successor of the accepted r5 remote runtime. Import/prepare are offline.

The original full-stack contract and acceptance are never altered. This entry
adds a separate, finite source delta and one independent remote service. It
reuses the original telemetry, typed QUERY, health and recorded-attempt code;
it does not replace any function inside those modules. Root owns live use.
"""
import argparse
import ast
from copy import deepcopy
import dis
import hashlib
import importlib
import json
import marshal
import os
from pathlib import Path
import re
import runpy
import stat
import subprocess
import sys
import time
import types
import xmlrpc.client

W = Path(__file__).resolve().parents[3]
REMOTE = '/remote_grasp6d_node'
PRODUCTION = 'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py'
SELF = '.runtime/r27_observation_planning_20261102/runtime_binding/remote_only_revision_r4.py'
TEST = '.runtime/r27_observation_planning_20261102/runtime_binding/tests/test_remote_only_revision_r4.py'
RUNTIME_PREFIX = '.runtime/r27_observation_planning_20261102/'
BASELINE_RELEASE_SHA = '9e483c892acfa1b9361a74873e90c972e7be0bfdb634f105b2b25fffaa92449a'
BASELINE_TYPED_ASSETS_SHA = '84c223af73021caf9a431cb11ac3d3f73e7059846c399933aadda23cfbedfc84'
BASELINE_TYPED_ACCEPTANCE_SHA = 'ee8c623caf1d0ec32f02fe66f116b1cf61186ba88457200aa4b425b12eee1670'
STACK = 'tools/full_stack_start.py'
ENTRY = 'tools/run_recorded_manual_grasp.py'
RECORDER = 'tools/record_grasp_keyframes_readonly.py'
RECORD_LAUNCHER = 'tools/run_recorded_grasp.py'
TYPED = 'tools/diagnostics/verify_physical_reference_startup.py'
RELAY = 'devel/lib/alicia_grasp_modes/mode_aware_remote_grasp6d.py'
MODE_ENTRY = 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
SELECTION = 'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py'
MODULE_SOURCES = {
    'alicia_grasp_modes.silhouette_observation_planning': PRODUCTION,
    'alicia_grasp_modes.observe_return_remote':
        'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
    'alicia_grasp_modes.observation_source_refresh':
        'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_source_refresh.py',
    'alicia_grasp_modes.reachable_observation_views':
        'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py',
}
PLACEMENT = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_joint_placement.py'
PLACEMENT_MODULE = 'alicia_flexible_grasp.grasp.contact_joint_placement'
PLACEMENT_BEFORE_SHA = 'bc541e074a4ef68828e7973abd87f5030744d6d8d697de8260eb05b0ffb30e31'
MODULE_SOURCES[PLACEMENT_MODULE] = PLACEMENT
PARENT_SELF = '.runtime/r27_observation_planning_20261102/runtime_binding/remote_only_revision_r3.py'
PARENT_TEST = '.runtime/r27_observation_planning_20261102/runtime_binding/tests/test_remote_only_revision_r3.py'
PARENT_HELPER_SHA = 'f9f6d3b29e0504556b5bf92d47a25ac5157a474ff985f7111f04cf8689237136'
ARCHIVE = Path('/home/zhuyupei/ssd_data/实验记录/Robot-Controller')
MAINTENANCE = ARCHIVE/'maintenance/r27_observation_planning_20261102'
PARENT_OUTPUT = ARCHIVE/'ros_log/full_latest_r26_manual_20261102_r1/r27_remote_only_refresh_fk5_20261102_r5'
PARENT_ACCEPTANCE = dict(path=str(PARENT_OUTPUT/'acceptance/result.json'),
    sha256='aa912771b4dabb979272e174d1e3eb5bc5e0f972646393873f207cea96512850')
PARENT_REVISION = dict(path=str(MAINTENANCE/'remote_only_prepare_r5_final/revision.json'),
    sha256='b18f5f04a17b3d7bf8f3ecbe897fa67dddfe52638595ca9db14b51b5a2c620ec')
PARENT_RELEASE = dict(path=str(MAINTENANCE/'remote_only_prepare_r5_final/release.json'),
    sha256='87bdf16a65d8bb289bdad314ddb9a1b381456fcd0af92137e17c1f918ce7b34d')
PARENT_REGRESSION_SHA = '6533fc7dd81057485a642b24aa302792b98109419323df28dd507cd031b54363'
NEW_TEST = 'src/alicia_flexible_grasp_supervisor/tests/test_contact_minimum_change_ray.py'
NEW_FIXTURE = 'src/alicia_flexible_grasp_supervisor/tests/fixtures/contact_minimum_ray_terminal_20261103.json'
BUSINESS_TESTS = frozenset({NEW_TEST,
    'src/alicia_flexible_grasp_supervisor/tests/test_contact_joint_placement.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_contact_joint_placement_branches.py'})
REQUIRED_SUCCESSOR_CHANGES = frozenset({PLACEMENT, NEW_TEST, NEW_FIXTURE, SELF, TEST, PARENT_TEST})
ALLOWED_SUCCESSOR_CHANGES = REQUIRED_SUCCESSOR_CHANGES | BUSINESS_TESTS
REGRESSION_WRAPPER = RUNTIME_PREFIX+'run_offline_regression.py'
ORIGINAL_RUNNER = '.runtime/verify_integrated_failure_repair.py'
REFRESH_BINDINGS = {
    'alicia_grasp_modes.silhouette_observation_planning': ('request_binding',),
    'alicia_grasp_modes.observe_return_remote': (
        'capture_materialization_configuration', 'registered_measurement',
        'materialize_measured_source', 'request_binding'),
}
DEFERRED_FK_NAMES = ('FK_PROPOSAL_BUDGET_SEC', 'generate_reachable_observation_views')
SIMULATION_POLICY_KEYS = ('/mujoco_digital_twin/unknown_lift_gate_enabled',
    '/mujoco_digital_twin/min_score', '/mujoco_digital_twin/server_url')
ABANDONED = frozenset({RUNTIME_PREFIX+'runtime_binding/compute_pair_revision.py',
                      RUNTIME_PREFIX+'runtime_binding/tests/test_compute_pair_revision.py'})
NO_ACTION = dict(cold_check_executed=False, temperature_queried=False,
                 torque_disable_commands_sent=False, motion_commands_sent=False,
                 mode_commands_sent=False, physical_grasp_verified=False)


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=sha(path))


def read_ref(reference):
    require(type(reference) is dict and set(reference) == {'path', 'sha256'}, 'exact file reference required')
    path = Path(reference['path'])
    require(path.is_absolute(), 'absolute evidence path required')
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == reference['sha256'], 'bound evidence changed: '+str(path))
    return json.loads(raw)


def load_source(relative, sources):
    path = W/relative
    require(relative in sources and sha(path) == sources[relative], 'helper source differs: '+relative)
    sys.path.insert(0, str(W/'tools'))
    module = types.ModuleType('_r27r4_'+path.stem)
    module.__file__ = str(path)
    sys.modules[module.__name__] = module
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    return module


def source_delta(before, after):
    return {p: dict(before_sha256=before.get(p), after_sha256=after.get(p))
            for p in sorted(set(before) | set(after)) if before.get(p) != after.get(p)}


def validate_source_delta(before, after, review):
    require(review.get('kind') == 'r27_minimum_placement_source_review' and review.get('reviewed') is True,
            'reviewed finite successor source inventory required')
    require(review.get('baseline_release') == PARENT_RELEASE
            and review.get('parent_acceptance') == PARENT_ACCEPTANCE, 'wrong accepted parent release')
    changed = source_delta(before, after)
    require(changed == review['changed_sources'] and after == review['source_sha256'],
            'successor review/source delta differs')
    require(REQUIRED_SUCCESSOR_CHANGES <= set(changed) <= ALLOWED_SUCCESSOR_CHANGES,
            'source delta exceeds finite placement successor or omits required coverage')
    require(all(row['after_sha256'] is not None for row in changed.values()), 'source removal forbidden')
    require(before.get(PLACEMENT) == PLACEMENT_BEFORE_SHA and before.get(PARENT_SELF) == PARENT_HELPER_SHA
            and after.get(PARENT_SELF) == PARENT_HELPER_SHA, 'parent placement/helper source differs')
    extras = review.get('additional_source_files')
    require(type(extras) is list and len(extras) == len(set(extras))
            and {SELF, TEST} <= set(extras) <= {SELF, TEST, PARENT_TEST},
            'exact finite successor adapter source list required')
    require(not ABANDONED.intersection(after), 'abandoned compute-pair adapter is not deployable')
    return changed


def sealed_release_evidence(release, run, entry, stack):
    """Verify historical phases/discovery from sealed bytes, not today's tree."""
    require(release.get('schema') == 1 and release.get('kind') == 'current_offline_release'
            and all(release.get(key) is True for key in ('passed', 'reviewed', 'installed'))
            and release.get('source_count') == len(release['source_sha256'])
            and run['source_sha256'] == release['source_sha256'], 'historical release/source binding differs')
    require(run.get('complete_scope_requested') is True and run.get('offline_scope_complete') is True
            and run.get('changed_sources') == [], 'historical full offline scope incomplete')
    summary = entry.recorded_case_summary(run, allow_skips=True)
    sources, files = run['source_sha256'], run['test_files']
    roots, explicit, exclusions = run['test_roots'], run['explicit_test_files'], run.get('excluded_files', [])
    relative = lambda path: type(path) is str and not Path(path).is_absolute() and '..' not in Path(path).parts
    require(all(relative(path) for path in sources), 'invalid historical source path')
    require(type(roots) is list and roots and len(roots) == len(set(roots))
            and all(relative(root) for root in roots)
            and all(any(Path(required) == Path(root) or Path(root) in Path(required).parents for root in roots)
                    for required in stack.CURRENT_TEST_ROOTS), 'historical test roots incomplete')
    require(type(explicit) is list and len(explicit) == len(set(explicit))
            and all(relative(path) and path in sources for path in explicit), 'historical explicit tests invalid')
    require(type(exclusions) is list and all(type(row) is dict and relative(row.get('path'))
            and type(row.get('reason')) is str and bool(row['reason']) and row['path'] not in files
            for row in exclusions) and len({row['path'] for row in exclusions}) == len(exclusions),
            'historical exclusions invalid')
    discovered = {path for path in sources if Path(path).name.startswith('test_') and Path(path).suffix == '.py'
                  and '__pycache__' not in Path(path).parts and any(Path(root) in Path(path).parents for root in roots)} | set(explicit)
    require(discovered == set(files) | {row['path'] for row in exclusions}, 'sealed historical discovery differs')
    if stack.HISTORICAL_PREWARM_TEST in sources:
        require(stack.HISTORICAL_PREWARM_TEST in files, 'historical prewarm coverage omitted')
    require(type(run.get('scope_name')) is str and bool(run['scope_name']), 'historical scope name missing')
    skipped = [dict(nodeid=row['nodeid'], reason=row['skip_reason']) for row in run['case_results']
               if row['setup'] == 'skipped' or row['call'] == 'skipped']
    complete = not skipped
    require(type(release.get('full_regression_passed')) is bool and release['full_regression_passed'] is complete
            and run.get('full_regression_passed') is complete, 'historical full-regression claim differs')
    return dict(current=release['actual_regression'], scope=run['scope_name'], current_summary=summary,
        exact_case_inventory=True, test_file_count=len(files), test_roots=roots, excluded_files=exclusions,
        skipped_cases=skipped, test_bodies_executed=complete, current_full_regression_passed=complete)


def validate_successor_cases(parent_run, run):
    previous, current = parent_run['expected_case_ids'], run['expected_case_ids']
    require(len(previous)==len(set(previous)) and len(current)==len(set(current))
            and set(previous)<=set(current), 'successor omitted or duplicated historical cases')
    def skips(value):
        return {(row['nodeid'],row.get('skip_reason')) for row in value['case_results']
                if row['setup']=='skipped' or row['call']=='skipped'}
    require(skips(parent_run)==skips(run), 'successor historical skip conditions changed')
    return dict(parent_case_count=len(previous), current_case_count=len(current),
                added_case_count=len(set(current)-set(previous)), historical_cases_preserved=True)


def verify_regression_review(review, regression_ref, run, binding, expected_entry=None):
    """Bind the unchanged R26-based wrapper review to the parent-based deployment review."""
    regression_review_ref=review['regression_source_review']
    regression_review=read_ref(regression_review_ref)
    require(regression_review.get('kind')=='r27_offline_source_review'
            and regression_review.get('reviewed') is True
            and regression_review['baseline_release']['sha256']==BASELINE_RELEASE_SHA,
            'original wrapper regression review differs')
    baseline=read_ref(regression_review['baseline_release'])
    require(regression_review['source_sha256']==review['source_sha256']==run['source_sha256']
            and regression_review['changed_sources']==source_delta(baseline['source_sha256'],run['source_sha256']),
            'regression and deployment review source inventories differ')
    extras={REGRESSION_WRAPPER,PARENT_SELF,PARENT_TEST,SELF,TEST}
    tests={PARENT_TEST,TEST}
    require(set(regression_review.get('additional_source_files',()))==extras
            and len(regression_review['additional_source_files'])==len(extras)
            and set(regression_review.get('additional_test_files',()))==tests
            and len(regression_review['additional_test_files'])==len(tests)
            and tests<=set(run['explicit_test_files']) and tests<=set(run['test_files']),
            'original and successor runtime scope must both be retained')
    path=Path(regression_ref['path']);name=path.parent.name
    require(path==ARCHIVE/'ros_log/integrated_regression'/name/'result.json',
            'regression result must be the exact original runner output')
    receipt_ref=ref(MAINTENANCE/'regression_entry_receipts'/(name+'.json'))
    require(expected_entry is None or expected_entry==receipt_ref, 'regression entry receipt changed')
    receipt=read_ref(receipt_ref); argv=receipt['argv']
    require(receipt['source_review']==regression_review_ref
            and receipt['entry']==dict(path=str(W/REGRESSION_WRAPPER),sha256=run['source_sha256'][REGRESSION_WRAPPER])
            and receipt['original_runner']==dict(path=str(W/ORIGINAL_RUNNER),sha256=run['source_sha256'][ORIGINAL_RUNNER])
            and receipt['storage_binding']==binding
            and set(receipt['manifest_extensions'])==extras and len(receipt['manifest_extensions'])==len(extras)
            and set(receipt['scope_extensions'])==tests and len(receipt['scope_extensions'])==len(tests)
            and receipt['declared_test_file_count']==len(run['test_files'])
            and receipt['source_count']==len(run['source_sha256'])
            and receipt.get('sandbox_and_reconciliation_unchanged') is True
            and receipt.get('all_original_test_files_preserved') is True
            and receipt.get('hardware_or_network_operations_authorized') is False,
            'actual regression wrapper invocation/provenance differs')
    require(type(argv) is list and argv.count('--name')==1 and argv[argv.index('--name')+1]==name
            and argv.count('--storage-backend')==1 and argv[argv.index('--storage-backend')+1]=='ssd'
            and '--pilot-file' not in argv and run.get('complete_scope_requested') is True
            and run.get('offline_scope_complete') is True and run.get('source_unchanged') is True
            and run.get('changed_sources')==[], 'full unchanged-source regression invocation required')
    return dict(regression_source_review=regression_review_ref,regression_entry_receipt=receipt_ref,
                regression=regression_ref, distinct_review_baselines_verified=True)


def expected_module_sources(sources):
    return {name: dict(path=str(W/path), sha256=sources[path])
            for name,path in MODULE_SOURCES.items()}


def validate_loaded_modules(modules, sources):
    require(set(modules) == set(MODULE_SOURCES), 'exact five preloaded modules required')
    for name,path in MODULE_SOURCES.items():
        module = modules[name]
        require(sys.modules.get(name) is module and Path(module.__file__).resolve() == W/path
                and sha(module.__file__) == sources[path], 'same-process source import differs: '+name)
    return expected_module_sources(sources)


def validate_entry_bindings(namespace, modules):
    """Check actual class imports without calling planning or ROS methods."""
    require(namespace.get('__name__') == '__main__' and namespace.get('__file__') == str(W/MODE_ENTRY),
            'original remote entry namespace differs')
    node = namespace['ModeAwareRemoteGrasp6DNode']
    bindings = {}
    for module_name, symbol in (
            ('alicia_grasp_modes.silhouette_observation_planning', 'SilhouetteObservationPlanningMixin'),
            ('alicia_grasp_modes.observe_return_remote', 'ObserveReturnRemoteMixin')):
        expected = getattr(modules[module_name], symbol)
        require(namespace.get(symbol) is expected and expected in node.__mro__,
                'actual remote class does not use preloaded mixin: '+symbol)
        bindings['ModeAwareRemoteGrasp6DNode.'+symbol] = module_name+'.'+symbol
    refresh = modules['alicia_grasp_modes.observation_source_refresh']
    for module_name,symbols in REFRESH_BINDINGS.items():
        for symbol in symbols:
            require(getattr(modules[module_name], symbol) is getattr(refresh, symbol),
                    'mixin does not use preloaded refresh function: '+module_name+'.'+symbol)
            bindings[module_name+'.'+symbol] = 'alicia_grasp_modes.observation_source_refresh.'+symbol
    method = node.__dict__['_joint_space_observation_candidates']
    require(method.__globals__ is namespace and method.__code__.co_filename == str(W/MODE_ENTRY),
            'FK candidate method is not from the actual original entry')
    imported, names = False, []
    for instruction in dis.get_instructions(method):
        if instruction.opname == 'IMPORT_NAME':
            imported = instruction.argval == 'alicia_grasp_modes.reachable_observation_views'
        elif imported and instruction.opname == 'IMPORT_FROM':
            names.append(instruction.argval)
        elif imported and instruction.opname == 'POP_TOP':
            imported = False
    require(tuple(names) == DEFERRED_FK_NAMES, 'actual FK method deferred source import differs')
    fk = modules['alicia_grasp_modes.reachable_observation_views']
    require(all(hasattr(fk, name) for name in DEFERRED_FK_NAMES), 'preloaded FK exports missing')
    return dict(placement_dependency=placement_entry_binding(namespace, modules),
                actual_class_and_refresh_bindings=bindings,
                deferred_fk_import=dict(module='alicia_grasp_modes.reachable_observation_views',
                    symbols=list(DEFERRED_FK_NAMES), resolves_to_preloaded_module=True,
                    executed_by_startup_observer=False))


def expected_entry_bindings():
    bindings = {'ModeAwareRemoteGrasp6DNode.'+symbol: module+'.'+symbol for module,symbol in (
        ('alicia_grasp_modes.silhouette_observation_planning', 'SilhouetteObservationPlanningMixin'),
        ('alicia_grasp_modes.observe_return_remote', 'ObserveReturnRemoteMixin'))}
    bindings.update({module+'.'+symbol: 'alicia_grasp_modes.observation_source_refresh.'+symbol
                     for module,symbols in REFRESH_BINDINGS.items() for symbol in symbols})
    return dict(placement_dependency=expected_placement_binding(),
                actual_class_and_refresh_bindings=bindings,
                deferred_fk_import=dict(module='alicia_grasp_modes.reachable_observation_views',
                    symbols=list(DEFERRED_FK_NAMES), resolves_to_preloaded_module=True,
                    executed_by_startup_observer=False))


def placement_entry_binding(namespace, modules):
    """Observe the actual inherited deferred imports before original main."""
    original = namespace['original']
    node = namespace['ModeAwareRemoteGrasp6DNode']
    base = original.RemoteGrasp6DNode
    require(base in node.__mro__, 'original remote class is not inherited')
    module = modules[PLACEMENT_MODULE]
    symbols = ('contact_joint_placement', 'joint_placement_policy_parameters')
    for symbol in symbols:
        function = getattr(module, symbol)
        require(isinstance(function, types.FunctionType) and function.__globals__ is module.__dict__
                and function.__code__.co_filename == str(W/PLACEMENT),
                'placement function is not from the preloaded source: '+symbol)
    methods = {'_contact_joint_window_proposal': 'contact_joint_placement',
               '_contact_height_recipe_parameters': 'joint_placement_policy_parameters'}
    for name, symbol in methods.items():
        method = base.__dict__[name]
        require(getattr(node, name) is method and method.__globals__ is original.__dict__,
                'actual remote placement method binding differs: '+name)
        imported, names = False, []
        for instruction in dis.get_instructions(method):
            if instruction.opname == 'IMPORT_NAME':
                imported = instruction.argval == PLACEMENT_MODULE
            elif imported and instruction.opname == 'IMPORT_FROM':
                names.append(instruction.argval)
            elif imported and instruction.opname == 'POP_TOP':
                imported = False
        require(names == [symbol], 'actual placement deferred import differs: '+name)
    return dict(module=PLACEMENT_MODULE, symbols=list(symbols),
                inherited_methods=methods, resolves_to_preloaded_module=True,
                executed_by_startup_observer=False)


def expected_placement_binding():
    return dict(module=PLACEMENT_MODULE,
        symbols=['contact_joint_placement', 'joint_placement_policy_parameters'],
        inherited_methods={'_contact_joint_window_proposal':'contact_joint_placement',
                           '_contact_height_recipe_parameters':'joint_placement_policy_parameters'},
        resolves_to_preloaded_module=True, executed_by_startup_observer=False)


def main_guard_line(source, filename):
    body = ast.parse(source, filename=filename).body
    final = body[-1]
    expected = ast.parse("if __name__ == '__main__':\n    pass\n").body[0].test
    require(isinstance(final, ast.If) and ast.dump(final.test) == ast.dump(expected),
            'original entry must retain its final main guard')
    return final.lineno


def entry_observer(relay_code, mode_code, guard_line, complete, set_trace):
    """Observe only the two frozen original code objects; never replace them."""
    state = dict(relay_seen=False, mode_seen=False, completed=False,
                 actual_relay_code=None, actual_mode_code=None)

    def trace(frame, event, arg):
        if frame.f_code is not state['actual_mode_code']:
            return None
        if event == 'line' and frame.f_lineno == guard_line:
            set_trace(None)
            complete(frame.f_globals, state)
            state['completed'] = True
            return None
        return trace

    def audit(event, args):
        if event != 'exec' or state['completed']:
            return
        code = args[0]
        if code.co_filename == relay_code.co_filename:
            require(code == relay_code and not state['relay_seen'], 'original relay compiled code differs/repeated')
            state['relay_seen'] = True
            state['actual_relay_code'] = code
        elif code.co_filename == mode_code.co_filename:
            require(code == mode_code and state['relay_seen'] and not state['mode_seen'],
                    'original remote compiled code differs or relay was not observed')
            state['mode_seen'], state['actual_mode_code'] = True, code
            set_trace(trace)
    return types.SimpleNamespace(audit=audit, trace=trace, state=state)


def directory_identity(path):
    path = Path(path)
    require(path.is_absolute() and path.resolve(strict=True) == path,
            'working directory must have an existing canonical absolute path')
    metadata = path.stat()
    require(stat.S_ISDIR(metadata.st_mode), 'working directory is not a directory')
    return dict(path=str(path), device=metadata.st_dev, inode=metadata.st_ino)


def process_working_directory(stack, identity):
    """Bind /proc cwd to a stable process and the same still-reachable directory."""
    before = stack.process(identity['pid'])
    require(all(before[key] == identity[key] for key in ('pid', 'parent_pid', 'start_ticks')),
            'working directory process identity changed')
    proc = Path('/proc')/str(identity['pid'])
    require(sha(proc/'cmdline') == identity['cmdline_sha256'], 'working directory process command changed')
    target = os.readlink(proc/'cwd')
    require(not target.endswith(' (deleted)'), 'working directory was deleted')
    result = directory_identity(target)
    actual = (proc/'cwd').stat()
    require((actual.st_dev, actual.st_ino) == (result['device'], result['inode'])
            and os.readlink(proc/'cwd') == target and directory_identity(target) == result,
            'process working directory no longer matches its path')
    require(stack.process(identity['pid']) == before and sha(proc/'cmdline') == identity['cmdline_sha256'],
            'working directory process changed during observation')
    return result


def validate_source_receipt(receipt, revision_ref, identity, sources, relay_ref, construction_ref, working_directory):
    require(receipt.get('kind') == 'r27_remote_source_start_r4' and receipt['revision'] == revision_ref
            and receipt['pid'] == identity['pid']
            and all(receipt['process'][k] == identity[k] for k in ('pid','parent_pid','start_ticks')),
            'source receipt belongs to another process/revision')
    require(receipt['entry'] == dict(path=str(W/SELF),sha256=sources[SELF]) and receipt['relay'] == relay_ref
            and receipt['mode_entry'] == dict(path=str(W/MODE_ENTRY),sha256=sources[MODE_ENTRY])
            and receipt['module_sources'] == expected_module_sources(sources)
            and receipt['dependency_bindings'] == expected_entry_bindings(),
            'actual entry/module dependency source receipt differs')
    require(receipt.get('construction') == construction_ref and receipt.get('working_directory') == working_directory,
            'source receipt working directory/construction differs')
    require(receipt.get('parent_acceptance')==PARENT_ACCEPTANCE
            and type(receipt.get('parent_shutdown')) is dict
            and receipt['parent_shutdown'].get('path')==str(Path(construction_ref['path']).parent/'parent_stopped.json'),
            'source receipt parent shutdown provenance differs')
    require(receipt.get('source_checked_before_original_main') is True
            and receipt.get('original_entry_observed_at_main_guard') is True
            and receipt.get('original_compiled_codes_match_frozen_sources') is True
            and receipt.get('temporary_entry_trace_removed_before_main') is True
            and receipt.get('memory_attestation') is False,
            'same-process original entry proof missing or overstated')
    compiled = receipt.get('actual_compiled_code_sha256')
    require(type(compiled) is dict and set(compiled) == {'relay','mode_entry'}
            and all(type(value) is str and re.fullmatch('[0-9a-f]{64}', value) for value in compiled.values()),
            'actual original exec code digests missing')


def validate_transition(before, after, remote_service, membership):
    """Only the remote may acquire a new process; all other identities are exact."""
    require(set(before) == set(after) and len(before) == 17 and REMOTE in before, 'exact 17-node graph required')
    require(all(after[n] == before[n] for n in before if n != REMOTE), 'preserved node identity changed')
    old, new = before[REMOTE], after[REMOTE]
    require(new['pid'] != old['pid'] and new['start_ticks'] != old['start_ticks'], 'remote process was not replaced')
    require(remote_service['ActiveState'] == 'active' and remote_service['SubState'] == 'running'
            and int(remote_service['MainPID']) == new['pid'] and membership is True,
            'new remote must be the independent unit MainPID in its exact cgroup')
    require(type(new['parent_pid']) is int and new['parent_pid'] > 1,
            'new remote parent must be separately recorded; the same user systemd parent is permitted')


def validate_gateway_aliases(aliases, gateway, expected, *, capture=False):
    """One original MoveIt helper name may share the retained gateway process."""
    require(type(aliases) is dict and len(aliases) <= 1 and all(
        type(name) is str and re.fullmatch('/move_group_commander_wrappers_[0-9]+', name)
        for name in aliases), 'gateway auxiliary name/count differs')
    for row in aliases.values():
        require(all(row[k] == gateway[k] for k in ('pid','parent_pid','start_ticks','cmdline_sha256')),
                'foreign gateway alias')
    if capture:
        require(expected is None, 'initial auxiliary capture cannot replace a frozen snapshot')
    else:
        require(type(expected) is dict and aliases == expected, 'frozen gateway aliases changed')
    return aliases


def validate_alias_construction(reference, construction, revision_ref, output, baseline_aliases, gateway):
    require(reference['path'] == str(Path(output)/'construction.json')
            and construction.get('kind') == 'r27_remote_only_construction_r4'
            and construction.get('revision') == revision_ref
            and construction.get('gateway_identity') == gateway
            and construction.get('baseline_auxiliary_nodes') == baseline_aliases,
            'current auxiliary construction provenance differs')
    return validate_gateway_aliases(construction['auxiliary_nodes'], gateway, None, capture=True)


def recorder_process_evidence(stack, pid):
    """Read one process without exposing its unrelated environment in receipts."""
    before = stack.process(pid)
    directory = Path('/proc')/str(pid)
    raw = (directory/'cmdline').read_bytes()
    environment = dict(x.decode().split('=', 1) for x in (directory/'environ').read_bytes().split(b'\0') if b'=' in x)
    require(stack.process(pid) == before, 'recorder chain process changed during observation')
    keys = ('GRASP_RECORD_DIR', 'GRASP_RECORD_ROOT', 'ROS_LOG_DIR', 'ALICIA_STORAGE_BACKEND',
            'ALICIA_STORAGE_IDENTITY', 'PYTHONNOUSERSITE', 'PYTHONDONTWRITEBYTECODE')
    return dict(identity=dict(before, cmdline_sha256=hashlib.sha256(raw).hexdigest()),
                argv=[part.decode() for part in raw.split(b'\0') if part],
                environment={key: environment.get(key) for key in keys})


def validate_recorder_chain(ready, manifest, record, archive_root, storage, config, processes, boot_id):
    """Require the original wrapper's live sibling recorder, not just a name."""
    record, archive_root = Path(record), Path(archive_root)
    wrapper, recorder, attempt_process = (processes[key] for key in ('wrapper', 'recorder', 'attempt'))
    parent, writer, child = (row['identity'] for row in (wrapper, recorder, attempt_process))
    for row in (parent, writer, child):
        require(type(row['pid']) is int and row['pid'] > 1 and type(row['parent_pid']) is int
                and row['parent_pid'] > 1 and type(row['start_ticks']) is str
                and row['start_ticks'].isascii() and row['start_ticks'].isdecimal()
                and int(row['start_ticks']) > 0, 'invalid recorder chain process identity')
    require(len({row['pid'] for row in (parent, writer, child)}) == 3
            and writer['parent_pid'] == child['parent_pid'] == parent['pid']
            and int(parent['start_ticks']) <= int(writer['start_ticks']) <= int(child['start_ticks']),
            'recorder and attempt must be original wrapper children in startup order')
    require(record.is_absolute() and record.parent == archive_root
            and type(ready['pid']) is int and ready['pid'] == writer['pid']
            and ready['record_dir'] == str(record) and ready['bag_path'] == str(record/'keyframes.bag')
            and isinstance(ready.get('started_at'), str) and bool(ready['started_at']),
            'READY recorder path/PID differs')
    metadata = manifest['metadata']
    require(manifest.get('schema') == 1 and manifest.get('record_id') == record.name
            and manifest.get('state') == 'recording' and manifest.get('recorder_closed') is False
            and manifest.get('completed_at') is None and manifest.get('local_data_deleted') is False
            and manifest['owner'] == dict(pid=parent['pid'], start_ticks=parent['start_ticks'], boot_id=boot_id)
            and type(metadata['owner_pid']) is int and metadata['owner_pid'] == parent['pid']
            and metadata['launcher'] == 'run_recorded_grasp.py' and metadata['storage'] == storage
            and type(metadata['recording_duration_seconds']) in (int, float)
            and metadata['recording_duration_seconds'] == 600.
            and metadata['command'] == attempt_process['argv'], 'current recording manifest/parent chain differs')
    argv = wrapper['argv']
    require(argv[0] == '/usr/bin/python3' and str(W/RECORD_LAUNCHER) in argv,
            'original recorder wrapper entry required')
    script_index = argv.index(str(W/RECORD_LAUNCHER))
    flags = argv[1:script_index]
    require('-B' in flags and len(flags) == len(set(flags)) and set(flags) <= {'-B', '-u'},
            'unmodified unoptimized recorder wrapper invocation required')
    arguments = argv[script_index+1:]
    require('--' in arguments, 'wrapper must directly own the current attempt command')
    separator = arguments.index('--'); options = arguments[:separator]
    require(len(options) == 6 and len(set(options[::2])) == 3
            and set(options[::2]) == {'--storage-backend', '--root', '--duration'}
            and arguments[separator+1:] == attempt_process['argv'], 'exact 600-second SSD wrapper command required')
    options = dict(zip(options[::2], options[1::2]))
    require(options['--storage-backend'] == 'ssd' and options['--root'] == str(archive_root)
            and options['--duration'] in ('600', '600.0'), 'recorder wrapper storage/duration differs')
    require(type(config.get('min_free_bytes')) is int and config['min_free_bytes'] >= 0
            and config.get('upload_enabled') is False and config.get('delete_enabled') is False
            and config.get('archive_policy') == 'local_only', 'original local-only recording configuration required')
    expected_argv = ['/usr/bin/python3', '-u', str(W/RECORDER), '--record-dir', str(record),
        '--output', str(record/'keyframes.bag'), '--duration', '600.0', '--defer-finalize',
        '--config', str(archive_root/'archive_config.json'), '--min-free-bytes', str(config['min_free_bytes'])]
    require(recorder['argv'] == expected_argv, 'original recorder argv/record/600-second binding differs')
    environment = recorder['environment']
    require(environment.get('GRASP_RECORD_DIR') == str(record)
            and environment.get('GRASP_RECORD_ROOT') == str(archive_root)
            and environment.get('ROS_LOG_DIR') == str(record/'recorder_ros')
            and environment.get('ALICIA_STORAGE_BACKEND') == 'ssd'
            and json.loads(environment.get('ALICIA_STORAGE_IDENTITY') or 'null') == storage
            and environment.get('PYTHONNOUSERSITE') == '1'
            and environment.get('PYTHONDONTWRITEBYTECODE') == '1', 'recorder environment/storage differs')


def validate_recorder_graph(system, name, node, process, expected=None):
    require(re.fullmatch('/grasp_keyframe_recorder_readonly_'+str(process['pid'])+'_[0-9]+', name)
            and {key: node[key] for key in process} == process, 'foreign or reused recorder ROS process')
    owned = [[entry for entry in table if name in entry[1]] for table in system]
    publications, subscriptions, services = ([entry[0] for entry in table] for table in owned)
    require(publications == ['/rosout'], 'recorder has non-log publications or is unavailable')
    standard = {name+'/get_loggers', name+'/set_logger_level'}
    require(set(services) == standard and len(services) == len(standard)
            and all(entry[1] == [name] for entry in owned[2]), 'recorder has nonstandard or shared services')
    require({'/grasp/state', '/alicia_d/accepted_joint_states'} <= set(subscriptions)
            and len(subscriptions) == len(set(subscriptions)), 'READY recorder subscriptions are incomplete')
    snapshot = dict(name=name, node=dict(node), publications=sorted(publications),
                    subscriptions=sorted(subscriptions), services=sorted(services))
    require(expected is None or snapshot == expected, 'frozen recorder ROS identity/graph changed')
    return snapshot


class RecorderBinding:
    """An attempt-only exception for one READY writer; never used for cleanup."""
    def __init__(self, revision, record, instruction_ref):
        self.revision, self.record, self.instruction_ref = revision, Path(record), instruction_ref
        self.ready_ref = ref(self.record/'recorder_ready.json')
        self.manifest_ref = ref(self.record/'manifest.json')
        self.config_ref = ref(revision.storage.archive_root/'archive_config.json')
        require(self.ready_ref['path'] == str(self.record/'recorder_ready.json')
                and self.manifest_ref['path'] == str(self.record/'manifest.json')
                and self.config_ref['path'] == str(revision.storage.archive_root/'archive_config.json'),
                'recorder evidence must be the current direct files')
        self.ready, self.manifest, self.config = (read_ref(row) for row in
            (self.ready_ref, self.manifest_ref, self.config_ref))
        require(type(self.ready.get('pid')) is int and self.ready['pid'] > 1, 'READY recorder PID invalid')
        self.pids = dict(wrapper=os.getppid(), recorder=self.ready['pid'], attempt=os.getpid())
        self.processes = {key: recorder_process_evidence(revision.stack, pid) for key,pid in self.pids.items()}
        self.source_sha256 = {path: revision.sources[path] for path in (RECORDER, RECORD_LAUNCHER, 'tools/grasp_archive.py')}
        self.owner = self.reference = None
        self.check()

    def check(self):
        require(os.getppid() == self.pids['wrapper'] and os.getpid() == self.pids['attempt'],
                'attempt parent changed since recorder binding')
        require(read_ref(self.ready_ref) == self.ready and read_ref(self.manifest_ref) == self.manifest
                and read_ref(self.config_ref) == self.config and not (self.record/'recorder_outcome.json').exists(),
                'recorder receipt/config changed or recording already ended')
        current = {key: recorder_process_evidence(self.revision.stack, pid) for key,pid in self.pids.items()}
        require(current == self.processes, 'frozen recorder/wrapper/attempt process changed')
        require(all(sha(W/path) == digest for path,digest in self.source_sha256.items()), 'recorder source changed')
        validate_recorder_chain(self.ready, self.manifest, self.record, self.revision.storage.archive_root,
            self.revision.binding, self.config, current, self.revision.contract['cold_boot_id'])

    def graph_owner(self, master, caller, system):
        self.check()
        names = {name for table in system for _,owners in table for name in owners}
        if self.owner is None:
            candidates = [name for name in names if re.fullmatch(
                '/grasp_keyframe_recorder_readonly_'+str(self.pids['recorder'])+'_[0-9]+', name)]
            require(len(candidates) == 1, 'exact one current READY recorder ROS owner required')
            name = candidates[0]
        else:
            name = self.owner['name']
            require(name in names, 'frozen READY recorder left the ROS graph')
        observed = validate_recorder_graph(system, name, self.revision.node(master,name,caller),
            self.processes['recorder']['identity'], self.owner)
        self.check()
        if self.owner is None:
            self.owner = observed
            self.reference = self.revision.save(self.revision.output/('recorder_binding_'+self.instruction_ref['sha256']+'.json'),
                dict(schema=1, kind='r27_recorded_attempt_recorder_binding_r4', revision=self.revision.reference,
                    instruction=self.instruction_ref, record_dir=str(self.record), recorder_ready=self.ready_ref,
                    manifest=self.manifest_ref, recording_configuration=self.config_ref, processes=self.processes,
                    source_sha256=self.source_sha256, ros_owner=self.owner, memory_attestation=False,
                    observed_ns=time.time_ns(), **NO_ACTION))
        else:
            read_ref(self.reference)
        return name


def validate_reset_response(response, frozen_selection, parse_selection):
    require(response.success is True, 'new remote selection reset failed')
    acknowledged=json.loads(response.message)
    require(type(acknowledged) is dict and acknowledged.get('ready') is True
            and parse_selection(acknowledged)==parse_selection(frozen_selection),
            'new remote selection reset identity differs')
    return acknowledged


def validate_reload_chain(reload_ref, deployed, revision_ref, output, execution_review_ref):
    require(reload_ref['path'] == str(Path(output)/'reload_result.json')
            and deployed.get('kind') == 'r27_remote_only_reload_r4' and deployed.get('passed') is True
            and deployed.get('revision') == revision_ref and deployed.get('execution_review') == execution_review_ref
            and deployed['source_startup_receipt']['path'] == str(Path(output)/'source_start.json')
            and deployed['construction']['path'] == str(Path(output)/'construction.json'),
            'exact current completed reload and execution review required')


def validate_acceptance_chain(acceptance_ref, accepted, revision_ref, output, release_ref,
                              contract_ref, execution_review_ref, reload_ref, deployed):
    validate_reload_chain(reload_ref, deployed, revision_ref, output, execution_review_ref)
    require(acceptance_ref['path']==str(Path(output)/'acceptance/result.json')
            and reload_ref['path']==str(Path(output)/'reload_result.json'),
            'acceptance/reload must be the exact current revision outputs')
    require(accepted.get('kind')=='r27_remote_only_acceptance_r4' and accepted.get('passed') is True
            and accepted.get('typed_initial_contract_ready') is True and accepted['revision']==revision_ref
            and accepted['source_manifest']==release_ref and accepted['contract']==contract_ref,
            'typed acceptance source/contract/revision chain differs')
    require(deployed.get('kind')=='r27_remote_only_reload_r4' and deployed.get('passed') is True
            and deployed['revision']==revision_ref and accepted['reload']==reload_ref,
            'typed acceptance is not bound to the actual completed reload')
    require(accepted['execution_review']==deployed['execution_review']==execution_review_ref,
            'reload/acceptance execution review differs')
    require(all(accepted[k]==deployed[k] for k in ('nodes','auxiliary_nodes','remote_service',
                                                'remote_startup_binding','source_startup_receipt','construction',
                                                'parent_acceptance','parent_shutdown'))
            and accepted['state']['epoch_ns']==deployed['state']['epoch_ns'],
            'typed acceptance does not retain the reload instances/epoch')


def verify_parent_chain(data, stack, entry, original):
    """Verify sealed accepted history; never assert its source tree is installed now."""
    references = {}
    def bound(reference):
        value = read_ref(reference)
        prior = references.setdefault(reference['path'], reference['sha256'])
        require(prior == reference['sha256'], 'parent path has conflicting digests')
        return value
    require(data.get('parent_acceptance') == PARENT_ACCEPTANCE
            and data.get('parent_revision') == PARENT_REVISION, 'exact accepted r5 parent required')
    parent, revision = bound(PARENT_ACCEPTANCE), bound(PARENT_REVISION)
    require(parent['revision'] == PARENT_REVISION and parent['source_manifest'] == PARENT_RELEASE
            and revision['release'] == PARENT_RELEASE and revision['runtime_output'] == str(PARENT_OUTPUT)
            and revision['entry'] == dict(path=str(W/PARENT_SELF),sha256=PARENT_HELPER_SHA),
            'accepted parent revision/release/entry differs')
    require(sha(W/PARENT_SELF) == PARENT_HELPER_SHA, 'immutable parent adapter changed')
    for key in ('baseline_acceptance','baseline_release','baseline_typed','baseline_startup','typed_assets'):
        require(data[key] == revision[key], 'successor changed original retained evidence: '+key)
        bound(revision[key])
    release = bound(PARENT_RELEASE)
    require(release['actual_regression']['sha256'] == PARENT_REGRESSION_SHA,
            'accepted parent regression differs')
    run = bound(release['actual_regression'])
    historical = original.sealed_release_evidence(release, run, entry, stack)
    require(parent['verification'] == historical, 'accepted parent sealed verification differs')
    review = bound(revision['source_review'])
    native_release = bound(revision['baseline_release'])
    original.validate_source_delta(native_release['source_sha256'], release['source_sha256'], review)
    require(release['supersedes'] == revision['baseline_release']
            and release['change_review'] == revision['source_review'], 'parent release derivation differs')
    reload_ref = parent['reload']
    deployed = bound(reload_ref)
    execution_ref = parent['execution_review']; execution = bound(execution_ref)
    require(execution.get('reviewed') is True and execution.get('kind') == 'r27_remote_only_execution_review_r3'
            and execution.get('revision') == PARENT_REVISION and execution.get('entry') == revision['entry']
            and execution.get('allowed_node_changes') == [REMOTE]
            and execution.get('cold_check_authorized') is False, 'parent execution review differs')
    original.validate_acceptance_chain(PARENT_ACCEPTANCE,parent,PARENT_REVISION,PARENT_OUTPUT,
        PARENT_RELEASE,parent['contract'],execution_ref,reload_ref,deployed)
    entry.validate_acceptance(parent)
    native = bound(revision['baseline_acceptance'])
    require(parent['contract'] == native['contract'], 'parent retained contract changed')
    contract = bound(parent['contract'])
    require(set(parent['nodes']) == set(native['nodes']) and len(parent['nodes']) == 17
            and all(parent['nodes'][name] == native['nodes'][name] for name in native['nodes'] if name != REMOTE)
            and parent['retained_node_count'] == 16, 'parent did not preserve the sixteen original nodes')
    require(all(parent[key] == native[key] for key in ('core_unit','core_service','master_identity','simulation'))
            and parent['state']['epoch_ns'] == native['state']['epoch_ns'], 'parent retained runtime/epoch differs')
    construction = bound(parent['construction'])
    source = bound(parent['source_startup_receipt'])
    working = construction['working_directory']
    require(construction.get('kind') == 'r27_remote_only_construction_r3'
            and construction['revision'] == PARENT_REVISION
            and construction['original_remote'] == native['nodes'][REMOTE]
            and construction['source_sha256'] == release['source_sha256']
            and construction['epoch_ns'] == parent['state']['epoch_ns']
            and construction['original_configuration'] == contract['configuration'],
            'parent construction provenance differs')
    original.validate_alias_construction(parent['construction'],construction,PARENT_REVISION,PARENT_OUTPUT,
        native['auxiliary_nodes'],native['nodes']['/motion_gateway'])
    require(construction['auxiliary_nodes'] == parent['auxiliary_nodes'], 'parent alias receipt differs')
    original.validate_source_receipt(source,PARENT_REVISION,parent['nodes'][REMOTE],release['source_sha256'],
        dict(path=str(W/RELAY),sha256=contract['current_sha256'][str(W/RELAY)]),parent['construction'],working)
    launch_ref=dict(path=str(PARENT_OUTPUT/'launch_intent.json'),
        sha256='688814b904e5f02ea124790cdac9300a80bed29e1ddb4945588dc3a0afee5819')
    launch=bound(launch_ref); service=parent['remote_service']
    command=launch['command']; argv=construction['new_argv']
    require(launch['unit']==service['unit'] and launch['construction']==parent['construction']
            and argv[:2]==['/usr/bin/python3',str(W/RELAY)]
            and len(argv)==4 and argv[2]=='__name:=remote_grasp6d_node'
            and argv[3]=='__log:='+str(PARENT_OUTPUT/'remote_node.log'), 'parent original relay argv differs')
    prefix=['systemd-run','--user','--collect','--unit='+service['unit'],'--property=Type=exec',
        '--property=Restart=no','--property=WorkingDirectory='+working['path'],
        '--property=StandardOutput=append:'+str(PARENT_OUTPUT/'remote.log'),
        '--property=StandardError=append:'+str(PARENT_OUTPUT/'remote.log'),
        '--setenv=PYTHONNOUSERSITE=1','--setenv=PYTHONDONTWRITEBYTECODE=1']
    require(command[:len(prefix)]==prefix, 'parent service launch options differ')
    child_argv=command[len(prefix):]
    require(len(child_argv)==9 and child_argv[:6]==['/usr/bin/python3','-B',str(W/PARENT_SELF),'child',
                '--private',str(PARENT_OUTPUT/'private_start.json')]
            and child_argv[6]=='--private-sha256' and re.fullmatch('[0-9a-f]{64}',child_argv[7])
            and child_argv[8]=='__name:=remote_grasp6d_node'
            and hashlib.sha256(b''.join(x.encode()+b'\0' for x in child_argv)).hexdigest()
                == parent['nodes'][REMOTE]['cmdline_sha256'], 'parent launcher process argv binding differs')
    require(service['properties']['MainPID']==str(parent['nodes'][REMOTE]['pid'])
            and service['properties']['ActiveState']=='active' and service['properties']['SubState']=='running'
            and bool(service['properties']['ControlGroup']) and bool(service['properties']['InvocationID']),
            'parent service process identity differs')
    remote_binding=bound(parent['remote_startup_binding'])
    require(remote_binding['node_identity']=={k:v for k,v in parent['nodes'][REMOTE].items() if k!='uri'}
            and remote_binding['parameters']==construction['parameters'], 'parent remote binding differs')
    health=bound(parent['remote_dependency'])
    require(health.get('passed') is True, 'parent health evidence failed')
    require(source['runtime_pycache_prefix']==str(PARENT_OUTPUT/'unused_python_cache'),
            'parent cache namespace differs')
    return dict(acceptance=parent,revision=revision,release=release,construction=construction,
        source_receipt=source,launch=launch,child_argv=child_argv,relay_argv=argv,
        working_directory=working,references=references,
        proof=dict(kind='accepted_r5_remote_parent_sealed_verification',
            parent_acceptance=PARENT_ACCEPTANCE,parent_revision=PARENT_REVISION,parent_release=PARENT_RELEASE,
            parent_regression=release['actual_regression'],historical_verification=historical,
            old_release_claimed_installed_now=False,original_adapter_modified=False,
            retained_node_count=16,physical_grasp_verified=False))


class Revision:
    def __init__(self, reference):
        self.reference, self.data = reference, read_ref(reference)
        d = self.data
        require(d.get('schema') == 1 and d.get('kind') == 'r27_remote_only_revision_r4', 'wrong revision kind')
        require(d['workspace'] == str(W) and d['entry']['path'] == str(W/SELF)
                and d['entry']['sha256'] == sha(__file__), 'revision entry/workspace differs')
        self.baseline = read_ref(d['baseline_acceptance'])
        self.old_release = read_ref(d['baseline_release'])
        require(d['baseline_release']['sha256'] == BASELINE_RELEASE_SHA
                and self.baseline['source_manifest'] == d['baseline_release'], 'baseline release differs')
        self.contract = read_ref(self.baseline['contract'])
        self.config = read_ref(self.contract['configuration'])
        self.startup = read_ref(d['baseline_startup'])
        self.prior_typed = read_ref(d['baseline_typed'])
        self.assets = read_ref(d['typed_assets'])
        self.release = read_ref(d['release'])
        self.review = read_ref(d['source_review'])
        self.instruction = read_ref(d['instruction'])
        self.sources = self.release['source_sha256']
        self.stack = load_source(STACK, self.sources)
        self.entry = load_source(ENTRY, self.sources)
        self.parent_adapter = load_source(PARENT_SELF, self.sources)
        self.parent_chain = verify_parent_chain(d,self.stack,self.entry,self.parent_adapter)
        self.parent = self.parent_chain['acceptance']
        self.typed = load_source(TYPED, self.sources)
        self.storage = load_source('tools/experiment_storage.py', self.sources).select('ssd')
        self.output = Path(d['runtime_output'])
        self.session = Path(self.contract['output_root'])
        self.binding = self.contract['storage']
        self.entry.validate_acceptance(self.baseline)
        require(self.baseline['verification'] == self.contract['verification']
                and self.contract['release'] == d['baseline_release'], 'original verification/release binding differs')
        require(self.prior_typed['passed'] is True and self.prior_typed['typed_initial_contract_ready'] is True
                and self.prior_typed['native_acceptance'] == d['baseline_acceptance'], 'paired typed/native baseline required')
        require(self.prior_typed['assets'] == d['typed_assets'], 'typed asset reference differs')
        require(self.baseline['contract'] == self.startup['contract']
                and self.startup.get('dispatched') is True, 'actual startup contract differs')
        require(self.output.parent == self.session and self.output not in (self.session,PARENT_OUTPUT),
                'new child of original session required')
        require(self.binding['backend'] == 'ssd', 'this revision preserves the selected SSD backend')
        require(self.instruction.get('source') == 'actual_user_message' and bool(self.instruction.get('text'))
                and self.instruction.get('repair_and_continue_authorized') is True,
                'actual repair/continue instruction required')
        validate_source_delta(self.parent_chain['release']['source_sha256'], self.sources, self.review)
        require(self.release.get('supersedes') == PARENT_RELEASE
                and self.release.get('change_review') == d['source_review'], 'new release derivation differs')
        require(self.release['driver_build'] == self.old_release['driver_build']
                and self.release['typed_assets'] == d['typed_assets'], 'driver or typed build changed')
        baseline_run=read_ref(self.old_release['actual_regression'])
        require(baseline_run['source_sha256']==self.old_release['source_sha256'], 'baseline regression source differs')
        self.entry.recorded_case_summary(baseline_run, allow_skips=True)
        self.stack.current_release_evidence(self.release)
        self.regression_review_evidence = verify_regression_review(self.review,self.release['actual_regression'],
            read_ref(self.release['actual_regression']),self.binding,d['regression_entry_receipt'])
        require(self.release.get('regression_entry_receipt')==d['regression_entry_receipt']
                and self.release.get('regression_source_review')==self.review['regression_source_review'],
                'release regression review provenance differs')
        self.case_continuity = validate_successor_cases(
            read_ref(self.parent_chain['release']['actual_regression']), read_ref(self.release['actual_regression']))
        self.stack.driver_build_evidence(self.release)
        self.preserved_typed_evidence = self.verify_preserved_typed_assets()
        require(not self.config['nodes'][REMOTE]['respawn'] and not self.config['nodes'][REMOTE]['required'],
                'remote must be independently replaceable without roslaunch respawn')
        self.health = load_source('.runtime/verify_active_remote_readiness_20261020.py', self.sources)
        self.selection = load_source(SELECTION, self.sources)
        self.verify_files()

    def verify_preserved_typed_assets(self):
        """Retain exact R26 typed proof while validating the R27 installed tree.

        The original verify_assets remains unchanged: its final native R26
        whole-tree/discovery check is intentionally a different contract. This
        finite bridge retains every sealed derivation relationship and current
        typed file check without claiming that R26 is still installed.
        """
        d, assets, typed = self.data, self.assets, self.typed
        require(d['typed_assets']['sha256'] == BASELINE_TYPED_ASSETS_SHA
                and d['baseline_typed']['sha256'] == BASELINE_TYPED_ACCEPTANCE_SHA
                and read_ref(d['typed_assets']) == assets and read_ref(d['baseline_typed']) == self.prior_typed,
                'exact historical paired typed evidence required')
        require(d['typed_assets'] == self.old_release['typed_assets'] == self.prior_typed['assets']
                == self.release['typed_assets'] and assets['actual_regression'] == self.old_release['actual_regression']
                and self.prior_typed['native_acceptance'] == d['baseline_acceptance']
                and self.baseline['source_manifest'] == d['baseline_release']
                and self.prior_typed.get('passed') is True and self.prior_typed.get('typed_initial_contract_ready') is True
                and self.prior_typed['entry'] == dict(path=str(W/TYPED),sha256=assets['source_sha256'][TYPED]),
                'preserved typed/native/release provenance differs')
        require(assets.get('schema') == 5 and assets.get('kind') == 'immutable_physical_reference_startup_identity'
                and assets.get('revision') == 'r26_physical_source_admission_python_v1'
                and all(assets.get(key) is False for key in ('runtime_loaded','physical_grasp_verified',
                                                           'automatic_execution_authorized')),
                'unsupported preserved R26 typed identity or claim')
        derivation = assets['python_source_derivation']
        previous_ref, base_ref = derivation['previous_assets'], derivation['base_release']
        require(previous_ref['sha256'] == typed.R26_PREVIOUS_ASSETS_SHA256
                and base_ref['sha256'] == typed.R26_BASE_RELEASE_SHA256, 'immutable R24 typed parents differ')
        previous, base = read_ref(previous_ref), read_ref(base_ref)
        for key in ('expected_nodes','topics','type_md5','required_parameters','memory_attestation',
                    'formal_build','checklist','baseline_assets','baseline_launch','post_build_source_updates'):
            require(assets[key] == previous[key], 'preserved typed contract/evidence changed: '+key)
        require(assets['baseline_assets']['sha256'] == typed.BASELINE_ASSETS_SHA256,
                'original typed baseline differs')
        typed.verify_storage_launch_derivation(assets, read_ref(assets['baseline_assets']))
        require(assets['current_sha256'] == previous['current_sha256'], 'preserved compiled/generated/startup artifacts changed')
        typed_delta = typed.r26_source_delta(previous['source_sha256'], assets['source_sha256'])
        require(set(assets['source_sha256']) == set(previous['source_sha256'])
                and set(typed_delta) == typed.R26_TYPED_CHANGED_SOURCES
                and derivation.get('typed_source_delta') == typed_delta, 'historical three-source typed delta differs')
        run = read_ref(assets['actual_regression'])
        historical = sealed_release_evidence(self.old_release, run, self.entry, self.stack)
        require(historical == self.baseline['verification'] == self.contract['verification'],
                'sealed historical native verification differs')
        full_sources = run['source_sha256']
        full_delta = typed.r26_source_delta(base['source_sha256'], full_sources)
        require(set(full_delta) == typed.R26_FULL_CHANGED_SOURCES
                and all(row['before_sha256'] is not None and row['after_sha256'] is not None for row in full_delta.values()),
                'historical six-source R26 delta differs')
        require(all(full_sources.get(path) == digest for path,digest in assets['source_sha256'].items())
                and full_sources.get(typed.LAUNCH_SOURCE) == assets['current_sha256'][str(W/typed.LAUNCH_SOURCE)],
                'historical typed/launch source differs from its full regression')
        review = read_ref(derivation['change_review'])
        require(review.get('reviewed') is True and review.get('source_scope') == 'r26_physical_source_admission_python_only'
                and review.get('base_release') == base_ref and review.get('base_assets') == previous_ref
                and review.get('actual_regression') == assets['actual_regression']
                and review.get('formal_build') == assets['formal_build']
                and review.get('source_delta') == full_delta == derivation.get('source_delta')
                and all(review.get(key) is True for key in ('typed_runtime_contract_unchanged',
                    'formal_cpp_build_unchanged','typed_artifacts_unchanged')), 'historical root typed review differs')
        build = read_ref(assets['formal_build']); read_ref(assets['checklist'])
        require(type(build.get('returncode')) is int and build['returncode'] == 0
                and build.get('formal_release_build') is True
                and all(base['source_sha256'].get(path) == digest == full_sources.get(path)
                        for path,digest in build['source_sha256'].items()), 'preserved formal C++ build/source differs')
        prior_run = read_ref(base['actual_regression'])
        prior_historical = sealed_release_evidence(base, prior_run, self.entry, self.stack)
        typed.verify_r26_case_continuity(prior_run, run, review)
        selected, _ = self.stack.driver_build_evidence(self.release)
        require(selected == assets['formal_build'] == self.old_release['driver_build'],
                'current release selected another formal typed build')
        for update in assets['post_build_source_updates']:
            targeted = read_ref(update['targeted_validation']); read_ref(update['review'])
            require(sha(update['patch']['path']) == update['patch']['sha256'], 'historical recorder patch changed')
            require(targeted.get('passed') is True
                    and targeted['actual_project_source_sha256'].get(str(W/update['path'])) == update['after_sha256']
                    and assets['source_sha256'].get(update['path']) == update['after_sha256'],
                    'historical recorder validation/source differs')
        self.verify_preserved_typed_files()
        return dict(schema=1,kind='r27_preserved_r26_typed_assets_verification',
            historical_assets=d['typed_assets'],historical_typed_acceptance=d['baseline_typed'],
            historical_native_acceptance=d['baseline_acceptance'],historical_release=d['baseline_release'],
            historical_regression=assets['actual_regression'],historical_verification=historical,
            historical_parent_release=base_ref,historical_parent_summary=prior_historical['current_summary'],
            historical_derivation_review=derivation['change_review'],formal_build=assets['formal_build'],
            checklist=assets['checklist'],post_build_source_updates=assets['post_build_source_updates'],
            current_release=d['release'],current_regression=self.release['actual_regression'],
            current_typed_source_count=len(assets['source_sha256']),current_typed_artifact_count=len(assets['current_sha256']),
            current_startup_input_count=len(self.contract['current_sha256']),
            historical_discovery_basis='sealed_source_inventory',old_release_claimed_installed_now=False,
            historical_derivation_verified=True,current_typed_files_verified=True,
            original_typed_validator_modified=False,memory_attestation=False,
            typed_runtime_query_performed_by_bridge=False,physical_grasp_verified=False)

    def verify_preserved_typed_files(self):
        require(all(self.sources.get(path) == digest for path,digest in self.assets['source_sha256'].items()),
                'preserved typed sources differ from current tested release')
        self.stack.source_check(self.assets['source_sha256'])
        for path,digest in self.assets['current_sha256'].items():
            require(W in Path(path).parents and sha(path) == digest, 'preserved typed artifact changed: '+path)

    def verify_files(self, manual_return=False):
        require(read_ref(self.reference) == self.data, 'revision bytes changed')
        for key in ('baseline_acceptance','baseline_release','baseline_startup','baseline_typed','typed_assets',
                    'release','source_review','instruction','parent_acceptance','parent_revision','regression_entry_receipt'):
            read_ref(self.data[key])
        read_ref(self.review['regression_source_review'])
        require(all(sha(path)==digest for path,digest in self.parent_chain['references'].items()),
                'sealed parent evidence changed')
        require(sha(W/PARENT_SELF)==PARENT_HELPER_SHA, 'immutable parent adapter changed')
        self.storage.validate(self.output, subtree=self.storage.ros_log_root, expected=self.binding)
        self.stack.storage(self.output, manual_return=manual_return, backend='ssd', expected=self.binding)
        require(read_ref(self.baseline['contract']) == self.contract, 'original contract changed')
        require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == self.contract['cold_boot_id'],
                'host boot differs from retained runtime')
        for path, digest in self.contract['current_sha256'].items():
            require(sha(path) == digest, 'original startup input changed: '+path)
        control = read_ref(self.contract['control'])
        require(control == self.contract['control_snapshot'] and control.get('state') == 'active', 'control snapshot changed/inactive')
        self.stack.source_check(self.sources)
        self.verify_preserved_typed_files()
        require(not Path(self.contract['pycache_prefix']).exists(), 'original unused Python cache was populated')
        require(not (self.output/'unused_python_cache').exists(), 'new unused Python cache was populated')
        require(sha(W/RELAY) == self.contract['current_sha256'][str(W/RELAY)], 'remote relay changed')

    def save(self, path, value):
        path = Path(path)
        self.storage.validate(path, subtree=self.storage.ros_log_root, expected=self.binding)
        raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
        with path.open('xb') as stream:
            stream.write(raw); stream.flush(); os.fsync(stream.fileno())
        require(path.read_bytes() == raw, 'new evidence readback differs')
        return ref(path)

    def simulation_policy(self, master):
        """Same bound gate in manual or automatic; never needed for safe cleanup."""
        return self.entry.read_bound_parameters(self.config['parameters'],master.getParam,
                                                SIMULATION_POLICY_KEYS)

    def live_policy(self, master):
        import yaml
        require(self.stack.configuration(session=self.session) == self.config, 'expanded original configuration changed')
        values = self.entry.read_bound_parameters(self.config['parameters'], master.getParam,
            ('/gui/joint_direct_mode','/alicia_d_driver_node/software_temperature_monitoring_enabled',
             '/alicia_d_driver_node/temperature_poll_rate_hz','/alicia_d_driver_node/self_check_poll_rate_hz',
             '/camera/simulate','/camera/fallback_to_simulation','/camera/mode_depth_preset',
             '/grasp/unknown_lift_after_close_enabled','/grasp/unknown_observe_return_max_observations'))
        values.update(self.simulation_policy(master))
        require(values['/gui/joint_direct_mode'] is True, 'manual mode required')
        require(self.entry.parameter_values_equal(master.getParam('/grasp'),
                self.entry.bound_parameter_tree(self.config['parameters'])['grasp']),
                'authoritative grasp configuration changed')
        require(master.getParam('/handeye') == yaml.safe_load(self.stack.HAND_EYE.read_text())['handeye'],
                'paired hand-eye changed')
        simulation=self.stack.load(self.stack.SIM)
        require(hashlib.sha256(master.getParam('/robot_description').encode()).hexdigest()
                == simulation.IDENTITY['candidate_model_sha256'], 'paired URDF changed')
        return values

    def node(self, master, name, caller):
        uri = master.lookupNode(name)
        reply = xmlrpc.client.ServerProxy(uri, transport=self.health.RpcTransport()).getPid(caller)
        require(reply[0] == 1, 'node PID unavailable: '+name)
        row = self.stack.process(reply[2])
        return dict(row, cmdline_sha256=sha(Path('/proc')/str(row['pid'])/'cmdline'), uri=uri)

    def alias_snapshot(self, reference):
        return validate_alias_construction(reference,read_ref(reference),self.reference,self.output,
            self.parent['auxiliary_nodes'],self.parent['nodes']['/motion_gateway'])

    def working_directory_snapshot(self, reference):
        construction = read_ref(reference)
        require(reference['path'] == str(self.output/'construction.json')
                and construction.get('kind') == 'r27_remote_only_construction_r4'
                and construction.get('revision') == self.reference
                and construction.get('original_remote') == self.parent['nodes'][REMOTE],
                'working directory construction provenance differs')
        directory = construction['working_directory']
        require(type(directory) is dict and set(directory) == {'path', 'device', 'inode'}
                and type(directory['path']) is str and type(directory['device']) is int
                and type(directory['inode']) is int and directory['device'] >= 0 and directory['inode'] > 0
                and directory_identity(directory['path']) == directory,
                'frozen working directory identity changed')
        return directory

    def parent_runtime(self):
        """Live parent check only before shutdown; subsequent phases use sealed history."""
        parent=self.parent; remote=parent['nodes'][REMOTE]; service=parent['remote_service']
        require(self.stack.properties(service['unit'])==service['properties']
                and self.stack.in_group(remote['pid'],service['properties']['ControlGroup']),
                'accepted parent service/cgroup changed')
        proc=Path('/proc')/str(remote['pid'])
        command=(proc/'cmdline').read_bytes()
        require([x.decode() for x in command.split(b'\0') if x]==self.parent_chain['child_argv']
                and hashlib.sha256(command).hexdigest()==remote['cmdline_sha256'],
                'accepted parent launcher argv changed')
        directory=process_working_directory(self.stack,remote)
        require(directory==self.parent_chain['working_directory'], 'accepted parent cwd changed')
        raw=(proc/'environ').read_bytes()
        environment=dict(x.decode().split('=',1) for x in raw.split(b'\0') if b'=' in x)
        expected=dict(PYTHONPYCACHEPREFIX=str(PARENT_OUTPUT/'unused_python_cache'),
            PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
            ROS_LOG_DIR=str(PARENT_OUTPUT/'remote_ros'),R27_REMOTE_REVISION_SHA256=PARENT_REVISION['sha256'])
        require(all(environment.get(k)==v for k,v in expected.items())
                and 'PYTHONOPTIMIZE' not in environment and 'PYTHONHOME' not in environment,
                'accepted parent cache/source/log environment changed')
        require(all(self.stack.process(remote['pid'])[k]==remote[k] for k in ('pid','parent_pid','start_ticks'))
                and (proc/'cmdline').read_bytes()==command and (proc/'environ').read_bytes()==raw,
                'accepted parent process changed during capture')
        return dict(parent_acceptance=PARENT_ACCEPTANCE,process=remote,service=service,
                    launcher_argv=self.parent_chain['child_argv'],relay_argv=self.parent_chain['relay_argv'],
                    working_directory=directory,environment_sha256=hashlib.sha256(raw).hexdigest())

    def parent_shutdown_snapshot(self, reference, construction_ref):
        stopped=read_ref(reference)
        require(reference['path']==str(self.output/'parent_stopped.json')
                and stopped.get('kind')=='r27_successor_parent_stopped_r4'
                and stopped.get('revision')==self.reference and stopped.get('construction')==construction_ref
                and stopped.get('parent_acceptance')==PARENT_ACCEPTANCE
                and stopped.get('parent_process')==self.parent['nodes'][REMOTE]
                and stopped.get('parent_service')==self.parent['remote_service']
                and stopped.get('process_exit_observed') is True
                and stopped.get('forced_termination') is False,
                'sealed parent shutdown binding differs')
        return stopped

    def graph(self, master, caller, remote=None, service=None, remote_absent=False,
              *, expected_aliases, capture_aliases=False, recorder=None):
        require(not capture_aliases or (remote is None and service is None and not remote_absent
                and expected_aliases is None), 'auxiliary capture is only allowed with the original remote')
        require(recorder is None or (remote is not None and service is not None and not remote_absent
                and not capture_aliases), 'recording exception requires an accepted current remote')
        core = self.stack.properties(self.parent['core_unit'])
        require(core == self.parent['core_service'] and self.stack.process(master.getPid()) == self.parent['master_identity'],
                'retained core/master changed')
        sim = self.stack.properties(self.startup['components']['simulation']['unit'])
        expected_sim = self.parent['simulation']
        require(sim['MainPID'] == str(expected_sim['pid']) and sim['InvocationID'] == expected_sim['invocation_id']
                and sim['ControlGroup'] == expected_sim['control_group'], 'retained simulation changed')
        system = master.getSystemState(); observed = {n for group in system for _, owners in group for n in owners}
        observed.discard(caller)
        if recorder is not None:
            observed.remove(recorder.graph_owner(master,caller,system))
        expected = set(self.parent['nodes']) - ({REMOTE} if remote_absent else set())
        extras = observed-expected
        require(expected <= observed and len(extras) <= 1 and all(re.fullmatch('/move_group_commander_wrappers_[0-9]+', n)
                for n in extras), 'runtime graph has missing or extra owners')
        nodes = {name:self.node(master,name,caller) for name in sorted(expected)}
        for name in expected-{REMOTE}:
            require(nodes[name] == self.parent['nodes'][name], 'retained node changed: '+name)
            require(self.stack.in_group(nodes[name]['pid'], core['ControlGroup']), 'retained node left original cgroup')
            environment=dict(x.decode().split('=',1) for x in
                (Path('/proc')/str(nodes[name]['pid'])/'environ').read_bytes().split(b'\0') if b'=' in x)
            require(environment.get('PYTHONPYCACHEPREFIX')==self.contract['pycache_prefix']
                    and environment.get('PYTHONDONTWRITEBYTECODE')=='1' and environment.get('PYTHONNOUSERSITE')=='1'
                    and environment.get('ROS_LOG_DIR')==str(self.session/'core_ros'),
                    'retained source/cache/log environment changed: '+name)
        if remote_absent:
            require(REMOTE not in observed, 'old remote still owns graph')
        elif remote is None:
            require(nodes[REMOTE] == self.parent['nodes'][REMOTE], 'original remote identity changed')
            self.parent_runtime()
        else:
            require(nodes[REMOTE] == remote, 'new remote identity changed')
            actual_service = self.stack.properties(service['unit'])
            require(actual_service == service['properties'], 'new remote service changed')
            validate_transition(self.parent['nodes'], nodes, actual_service,
                                self.stack.in_group(remote['pid'],actual_service['ControlGroup']))
            source_start=read_ref(ref(self.output/'source_start.json'))
            self.parent_shutdown_snapshot(source_start['parent_shutdown'],source_start['construction'])
            working_directory=self.working_directory_snapshot(source_start['construction'])
            validate_source_receipt(source_start,self.reference,remote,self.sources,
                dict(path=str(W/RELAY),sha256=self.contract['current_sha256'][str(W/RELAY)]),
                source_start['construction'],working_directory)
            require(process_working_directory(self.stack,remote)==working_directory,
                    'new remote working directory differs from construction')
            require(subprocess.check_output(['systemctl','--user','show',service['unit'],
                    '--property=WorkingDirectory','--value'],text=True,timeout=4.) == working_directory['path']+'\n',
                    'new remote unit working directory differs from construction')
            environment=dict(x.decode().split('=',1) for x in
                (Path('/proc')/str(remote['pid'])/'environ').read_bytes().split(b'\0') if b'=' in x)
            require(environment.get('PYTHONPYCACHEPREFIX')==str(self.output/'unused_python_cache')
                    and environment.get('PYTHONDONTWRITEBYTECODE')=='1' and environment.get('PYTHONNOUSERSITE')=='1'
                    and environment.get('ROS_LOG_DIR')==str(self.output/'remote_ros')
                    and environment.get('R27_REMOTE_REVISION_SHA256')==self.reference['sha256'],
                    'new remote source/cache/log environment differs')
        aliases = {n:self.node(master,n,caller) for n in extras}
        if remote is None and not remote_absent:
            require(aliases==self.parent['auxiliary_nodes'], 'accepted parent aliases changed')
        validate_gateway_aliases(aliases,nodes['/motion_gateway'],expected_aliases,capture=capture_aliases)
        publishers = dict(system[0])
        owners = {'/alicia_d/accepted_joint_states':'/alicia_d_driver_node',
            '/grasp/state':'/grasp_task_node','/alicia_d/control_reference_epoch':'/alicia_d_driver_node',
            '/alicia_d/motion_enabled':'/alicia_d_driver_node','/alicia_d/actuation_status':'/alicia_d_driver_node',
            self.config['parameters']['/camera/color_topic']:'/supervisor_camera',
            self.config['parameters']['/camera/depth_topic']:'/supervisor_camera'}
        require(all(publishers.get(p) == [n] for p,n in owners.items()), 'retained feedback/camera ownership differs')
        return nodes, aliases


def wait_manual(revision, monitor, rospy, epoch=None, positions=None, timeout=8.):
    deadline, error = time.monotonic()+timeout, None
    while time.monotonic() < deadline:
        try:
            value = monitor.sample(revision.startup['started_ns'])
            require(epoch is None or value['epoch_ns'] == epoch, 'driver epoch changed during remote-only transition')
            if positions is not None:
                require(value['names'] == positions['names'] and max(abs(a-b) for a,b in
                    zip(value['positions'],positions['positions'])) <= .004, 'robot moved during remote-only transition')
            with monitor.lock:
                state = monitor.values['task'][1]
            require(str(state.state) == 'IDLE', 'reload requires exact IDLE task state')
            return value
        except RuntimeError as exc:
            error = str(exc); time.sleep(.025)
    raise RuntimeError('manual stationary snapshot unavailable: '+str(error))


def remote_graph(master):
    return [sorted(topic for topic,owners in group if REMOTE in owners) for group in master.getSystemState()]


def prepare(args):
    baseline_ref, typed_ref, regression_ref, review_ref, instruction_ref = (
        dict(path=str(getattr(args,k).resolve()),sha256=getattr(args,k+'_sha256'))
        for k in ('baseline','typed','regression','source_review','instruction'))
    baseline, prior_typed, run, review = (read_ref(r) for r in (baseline_ref,typed_ref,regression_ref,review_ref))
    require(baseline_ref==PARENT_ACCEPTANCE and baseline['revision']==PARENT_REVISION,
            'prepare requires exact immediate accepted parent')
    parent_revision=read_ref(PARENT_REVISION)
    require(typed_ref==parent_revision['baseline_typed'], 'original paired typed acceptance changed')
    release_ref = baseline['source_manifest']; previous = read_ref(release_ref)
    require(release_ref == PARENT_RELEASE, 'wrong accepted parent release')
    validate_source_delta(previous['source_sha256'],run['source_sha256'],review)
    require(review['baseline_release'] == release_ref, 'review uses another baseline')
    require(run.get('offline_scope_complete') is True and run.get('complete_scope_requested') is True,
            'completed new full offline scope required')
    require(BUSINESS_TESTS | {TEST} <= set(run['test_files']),
            'placement business tests and successor adapter test must be executed')
    require(PARENT_TEST in run['test_files'], 'historical r3 adapter cases must remain in scope')
    validate_successor_cases(read_ref(previous['actual_regression']),run)
    require(set(review.get('additional_test_files', [])) <= set(run['explicit_test_files'])
            and TEST in run['explicit_test_files'], 'extra runtime tests must retain explicit result provenance')
    sources = run['source_sha256']; stack = load_source(STACK,sources)
    storage = load_source('tools/experiment_storage.py',sources).select('ssd')
    contract = read_ref(baseline['contract']); binding = contract['storage']
    regression_proof=verify_regression_review(review,regression_ref,run,binding)
    out = args.output.resolve()
    storage.validate(out, subtree=storage.maintenance_root, expected=binding)
    require(not out.exists(), 'preparation output already exists')
    require(Path(args.runtime_child).name == args.runtime_child and args.runtime_child not in ('.','..'),
            'one new runtime child name required')
    instruction = read_ref(instruction_ref)
    require(instruction.get('source') == 'actual_user_message' and instruction.get('repair_and_continue_authorized') is True,
            'actual repair/continue instruction required')
    release = dict(schema=1,kind='current_offline_release',generation='R27_REMOTE_ONLY_MINIMUM_PLACEMENT_SUCCESSOR',
        created_ns=time.time_ns(),passed=True,reviewed=True,installed=True,
        installed_meaning='frozen source bytes on disk only; no runtime or physical result',
        actual_regression=regression_ref,source_sha256=sources,source_count=len(sources),
        full_regression_passed=run['full_regression_passed'],offline_scope_complete=True,
        driver_build=previous['driver_build'],typed_assets=previous['typed_assets'],supersedes=release_ref,
        change_review=review_ref,physical_grasp_verified=False,runtime_loaded=False,storage_binding=binding)
    release.update(regression_entry_receipt=regression_proof['regression_entry_receipt'],
                   regression_source_review=regression_proof['regression_source_review'])
    stack.current_release_evidence(release); stack.driver_build_evidence(release)
    out.mkdir()
    def save(name,value):
        storage.validate(out/name, expected=binding)
        raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
        with (out/name).open('xb') as f: f.write(raw); f.flush(); os.fsync(f.fileno())
        require((out/name).read_bytes()==raw,'preparation readback differs')
        return ref(out/name)
    new_release_ref=save('release.json',release)
    data=dict(schema=1,kind='r27_remote_only_revision_r4',workspace=str(W),entry=ref(__file__),
        parent_acceptance=PARENT_ACCEPTANCE,parent_revision=PARENT_REVISION,
        regression_entry_receipt=regression_proof['regression_entry_receipt'],
        baseline_acceptance=parent_revision['baseline_acceptance'],baseline_release=parent_revision['baseline_release'],
        baseline_typed=typed_ref,baseline_startup=parent_revision['baseline_startup'],typed_assets=prior_typed['assets'],
        release=new_release_ref,source_review=review_ref,instruction=instruction_ref,
        runtime_output=str(Path(contract['output_root'])/args.runtime_child),
        changed_node=REMOTE,retained_node_count=16,storage=binding,**NO_ACTION)
    plan_ref=save('revision.json',data)
    verified=Revision(plan_ref)
    typed_evidence_ref=save('preserved_typed_validation.json',dict(verified.preserved_typed_evidence,revision=plan_ref))
    print(json.dumps(dict(prepared=True,revision=plan_ref,preserved_typed_validation=typed_evidence_ref,
                         starts_services=False),ensure_ascii=False))


def load_live(args):
    require(not sys.flags.optimize and sys.flags.no_user_site, 'unoptimized Python with PYTHONNOUSERSITE=1 required')
    revision=Revision(dict(path=str(args.revision.resolve()),sha256=args.revision_sha256))
    review_ref=dict(path=str(args.execution_review.resolve()),sha256=args.execution_review_sha256)
    review=read_ref(review_ref)
    require(review.get('reviewed') is True and review.get('kind') == 'r27_remote_only_execution_review_r4'
            and review.get('revision') == revision.reference and review.get('entry') == ref(__file__),
            'independent exact revision execution review required')
    require(review.get('allowed_node_changes') == [REMOTE] and review.get('cold_check_authorized') is False,
            'execution scope differs')
    return revision,review_ref


def child(args):
    """Preload five frozen modules and observe the unchanged original relay."""
    private=Path(args.private); data=read_ref(dict(path=str(private),sha256=args.private_sha256))
    require(data['entry'] == ref(__file__), 'child helper changed')
    marker=data['revision']['sha256']
    if os.environ.get('R27_REMOTE_REVISION_SHA256') != marker:
        environment=dict(data['environment'], R27_REMOTE_REVISION_SHA256=marker)
        for key in ('INVOCATION_ID','JOURNAL_STREAM','SYSTEMD_EXEC_PID'):
            if key in os.environ:environment[key]=os.environ[key]
        os.execve('/usr/bin/python3',['/usr/bin/python3','-B',str(W/SELF),*sys.argv[1:]],environment)
    revision=Revision(data['revision']); revision.verify_files()
    working_directory=revision.working_directory_snapshot(data['construction'])
    revision.parent_shutdown_snapshot(data['parent_shutdown'],data['construction'])
    require(data['cwd'] == working_directory['path'] and data['argv'][0] == '/usr/bin/python3'
            and data['argv'][1] == str(W/RELAY), 'original remote argv/cwd differs')
    identity=dict(revision.stack.process(os.getpid()),cmdline_sha256=sha(Path('/proc')/str(os.getpid())/'cmdline'))
    require(process_working_directory(revision.stack,identity)==working_directory,
            'child initial working directory differs from construction')
    require(data['environment']['PYTHONDONTWRITEBYTECODE']=='1' and data['environment']['PYTHONNOUSERSITE']=='1'
            and data['environment']['PYTHONPYCACHEPREFIX']==str(revision.output/'unused_python_cache'),
            'new bytecode isolation differs')
    # The full original environment is private and is never copied into a public receipt.
    private.unlink()
    os.chdir(data['cwd'])
    sys.dont_write_bytecode=True;sys.pycache_prefix=data['environment']['PYTHONPYCACHEPREFIX']
    modules={name:importlib.import_module(name) for name in MODULE_SOURCES}
    validate_loaded_modules(modules,revision.sources)
    relay_raw, mode_raw=(W/RELAY).read_bytes(),(W/MODE_ENTRY).read_bytes()
    require(hashlib.sha256(relay_raw).hexdigest()==revision.contract['current_sha256'][str(W/RELAY)]
            and hashlib.sha256(mode_raw).hexdigest()==revision.sources[MODE_ENTRY], 'original entry source changed')
    relay_code=compile(relay_raw,str(W/RELAY),'exec',dont_inherit=True)
    mode_code=compile(mode_raw,str(W/MODE_ENTRY),'exec',dont_inherit=True)
    guard=main_guard_line(mode_raw,str(W/MODE_ENTRY))
    require(sys.gettrace() is None,'unexpected existing Python trace at original entry')

    def complete(namespace,state):
        require(sys.gettrace() is None,'temporary entry observer was not removed')
        module_sources=validate_loaded_modules(modules,revision.sources)
        dependencies=validate_entry_bindings(namespace,modules)
        require(state['relay_seen'] and state['mode_seen'] and sha(W/MODE_ENTRY)==revision.sources[MODE_ENTRY],
                'actual original entry source changed during load')
        require(revision.working_directory_snapshot(data['construction'])==working_directory
                and process_working_directory(revision.stack,identity)==working_directory,
                'child working directory changed before original main')
        receipt=dict(kind='r27_remote_source_start_r4',revision=revision.reference,pid=os.getpid(),
            process=revision.stack.process(os.getpid()),entry=ref(__file__),relay=ref(W/RELAY),
            construction=data['construction'],working_directory=working_directory,
            parent_acceptance=PARENT_ACCEPTANCE,parent_shutdown=data['parent_shutdown'],
            mode_entry=ref(W/MODE_ENTRY),module_sources=module_sources,dependency_bindings=dependencies,
            source_checked_before_original_main=True,original_entry_observed_at_main_guard=True,
            original_compiled_codes_match_frozen_sources=True,temporary_entry_trace_removed_before_main=True,
            actual_compiled_code_sha256={name:hashlib.sha256(marshal.dumps(state[key])).hexdigest()
                for name,key in (('relay','actual_relay_code'),('mode_entry','actual_mode_code'))},
            memory_attestation=False,runtime_pycache_prefix=sys.pycache_prefix,recorded_ns=time.time_ns())
        validate_source_receipt(receipt,revision.reference,receipt['process'],revision.sources,
            dict(path=str(W/RELAY),sha256=revision.contract['current_sha256'][str(W/RELAY)]),
            data['construction'],working_directory)
        revision.save(revision.output/'source_start.json',receipt)

    observer=entry_observer(relay_code,mode_code,guard,complete,sys.settrace)
    sys.addaudithook(observer.audit)
    sys.argv=data['argv'][1:]
    try:
        runpy.run_path(str(W/RELAY),run_name='__main__')
    finally:
        sys.settrace(None)
    require(observer.state['completed'],'original remote entry did not reach verified main guard')


def reload_remote(args):
    revision,review_ref=load_live(args)
    require(not revision.output.exists(),'single-use remote revision output required')
    revision.output.mkdir()
    os.environ.update(revision.storage.environment(revision.output/'observer_ros',session=revision.session))
    import rospy
    import rosgraph
    from std_srvs.srv import Trigger
    rospy.init_node('r27_remote_only_reload_r4',anonymous=True,disable_rosout=True)
    master=rosgraph.Master(rospy.get_name());monitor=revision.stack.Monitor(revision.config)
    report=dict(schema=1,kind='r27_remote_only_reload_r4',passed=False,revision=revision.reference,
        execution_review=review_ref,preserved_typed_assets=revision.preserved_typed_evidence,
        started_ns=time.time_ns(),**NO_ACTION)
    private=None
    try:
        revision.verify_files();revision.live_policy(master)
        before_nodes,before_aliases=revision.graph(master,rospy.get_name(),
            expected_aliases=None,capture_aliases=True)
        before_remote_graph=remote_graph(master)
        before=wait_manual(revision,monitor,rospy)
        selection=master.getParam('/grasp_mode/selection');grasp=master.getParam('/grasp')
        old=before_nodes[REMOTE];proc=Path('/proc')/str(old['pid'])
        original_environment=dict(x.decode().split('=',1) for x in (proc/'environ').read_bytes().split(b'\0') if b'=' in x)
        original_argv=[x.decode() for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
        parent_runtime=revision.parent_runtime()
        require(original_argv==parent_runtime['launcher_argv'], 'accepted parent launcher differs')
        relay_argv=list(parent_runtime['relay_argv'])
        working_directory=process_working_directory(revision.stack,old)
        environment=dict(original_environment)
        environment.update(revision.storage.environment(revision.output/'remote_ros',session=revision.session))
        environment.update(PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
            PYTHONPYCACHEPREFIX=str(revision.output/'unused_python_cache'),PYTHONUNBUFFERED='1',
            R27_REMOTE_REVISION_SHA256=revision.reference['sha256'])
        environment.pop('PYTHONOPTIMIZE',None);environment.pop('PYTHONHOME',None)
        for key in ('INVOCATION_ID','JOURNAL_STREAM','SYSTEMD_EXEC_PID','NOTIFY_SOCKET','WATCHDOG_PID','WATCHDOG_USEC'):
            environment.pop(key,None)
        argv=[x if not x.startswith('__log:=') else '__log:='+str(revision.output/'remote_node.log') for x in relay_argv]
        construction=dict(kind='r27_remote_only_construction_r4',revision=revision.reference,
            original_remote=old,original_argv=original_argv,new_argv=argv,
            parent_acceptance=PARENT_ACCEPTANCE,parent_revision=PARENT_REVISION,
            parent_runtime=parent_runtime,parent_relay_argv=relay_argv,
            working_directory=working_directory,
            auxiliary_nodes=before_aliases,gateway_identity=before_nodes['/motion_gateway'],
            baseline_auxiliary_nodes=revision.parent['auxiliary_nodes'],
            original_environment_sha256=hashlib.sha256((proc/'environ').read_bytes()).hexdigest(),
            changed_environment_keys=sorted(k for k in set(environment)|set(original_environment)
                if environment.get(k)!=original_environment.get(k)),
            parameters=revision.config['remote_parameters'],grasp=grasp,selection=selection,epoch_ns=before['epoch_ns'],
            original_configuration=revision.contract['configuration'],source_sha256=revision.sources,
            before=before,recorded_ns=time.time_ns(),**NO_ACTION)
        construction_ref=revision.save(revision.output/'construction.json',construction)
        require(revision.alias_snapshot(construction_ref)==before_aliases,'saved auxiliary snapshot changed')
        revision.verify_files();revision.live_policy(master)
        revision.graph(master,rospy.get_name(),expected_aliases=before_aliases)
        wait_manual(revision,monitor,rospy,before['epoch_ns'],before)
        require(revision.working_directory_snapshot(construction_ref)==working_directory
                and process_working_directory(revision.stack,old)==working_directory,
                'original remote working directory changed before shutdown')
        require(revision.parent_runtime()==parent_runtime
                and hashlib.sha256((proc/'environ').read_bytes()).hexdigest()==construction['original_environment_sha256'],
                'accepted parent changed before shutdown')
        code,_,_=xmlrpc.client.ServerProxy(old['uri'],transport=revision.health.RpcTransport()).shutdown(
            rospy.get_name(),'reviewed R27 remote-only source revision')
        require(code==1,'old remote shutdown rejected')
        deadline=time.monotonic()+20
        while proc.exists() and time.monotonic()<deadline: time.sleep(.05)
        require(not proc.exists(),'old remote did not exit; no force kill or duplicate launch')
        stopped_ref=revision.save(revision.output/'parent_stopped.json',dict(
            kind='r27_successor_parent_stopped_r4',revision=revision.reference,construction=construction_ref,
            parent_acceptance=PARENT_ACCEPTANCE,parent_process=old,parent_service=revision.parent['remote_service'],
            process_exit_observed=True,forced_termination=False,recorded_ns=time.time_ns(),**NO_ACTION))
        revision.parent_shutdown_snapshot(stopped_ref,construction_ref)
        revision.graph(master,rospy.get_name(),remote_absent=True,expected_aliases=before_aliases)
        wait_manual(revision,monitor,rospy,before['epoch_ns'],before)
        private=revision.output/'private_start.json'
        require(revision.working_directory_snapshot(construction_ref)==working_directory,
                'working directory changed before new remote launch')
        payload=dict(entry=ref(__file__),revision=revision.reference,argv=argv,environment=environment,
                     cwd=working_directory['path'],construction=construction_ref,parent_shutdown=stopped_ref)
        fd=os.open(private,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as f:json.dump(payload,f);f.flush();os.fsync(f.fileno())
        unit='alicia-r27-remote-'+str(time.time_ns())+'.service'
        command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec','--property=Restart=no',
            '--property=WorkingDirectory='+working_directory['path'],'--property=StandardOutput=append:'+str(revision.output/'remote.log'),
            '--property=StandardError=append:'+str(revision.output/'remote.log'),
            '--setenv=PYTHONNOUSERSITE=1','--setenv=PYTHONDONTWRITEBYTECODE=1',
            '/usr/bin/python3','-B',str(W/SELF),'child','--private',str(private),'--private-sha256',sha(private),
            *[x for x in argv[2:] if x.startswith('__name:=')]]
        revision.save(revision.output/'launch_intent.json',dict(unit=unit,command=command,construction=construction_ref))
        subprocess.run(command,check=True,capture_output=True,timeout=15)
        deadline=time.monotonic()+40;last=None
        while time.monotonic()<deadline:
            try:
                remote=revision.node(master,REMOTE,rospy.get_name())
                service=dict(unit=unit,properties=revision.stack.properties(unit))
                nodes,aliases=revision.graph(master,rospy.get_name(),remote,service,expected_aliases=before_aliases)
                source_start=read_ref(ref(revision.output/'source_start.json'))
                require(source_start['pid']==remote['pid'],'source preload PID differs')
                require(remote_graph(master)==before_remote_graph,'new remote graph is still initializing/differs')
                break
            except (OSError,RuntimeError,rosgraph.MasterError) as exc:last=repr(exc);time.sleep(.1)
        else:raise RuntimeError('new remote did not become ready: '+str(last))
        require(dict(master.getSystemState()[2]).get('/grasp_mode/reset_remote')==[REMOTE], 'reset owner differs')
        wait_manual(revision,monitor,rospy,before['epoch_ns'],before)
        answer=rospy.ServiceProxy('/grasp_mode/reset_remote',Trigger)()
        acknowledgement=validate_reset_response(answer,selection,revision.selection.parse_selection)
        revision.live_policy(master)
        require(master.getParam('/grasp_mode/selection')==selection and master.getParam('/grasp')==grasp,
                'selection/configuration changed during reload')
        prefix=(revision.output/'remote.log').read_bytes()
        markers=[s for s in prefix.decode(errors='replace').splitlines() if '[RemoteGrasp6D] authoritative /grasp loaded ' in s]
        require(len(markers)==1,'new remote authoritative startup marker missing/repeated')
        marker=re.search(r'key_count=(\d+) contact_height_registration_margin_enabled=(\S+) handeye_translation_max_error_m=(\S+)',markers[0])
        require(marker and int(marker.group(1))==len(grasp)
                and ast.literal_eval(marker.group(2))==grasp.get('contact_height_registration_margin_enabled')
                and ast.literal_eval(marker.group(3))==grasp.get('handeye_translation_max_error_m'),
                'new remote authoritative startup parameters differ')
        prefix_path=revision.output/'remote_startup_prefix.log'
        with prefix_path.open('xb') as f:f.write(prefix);f.flush();os.fsync(f.fileno())
        old_binding=read_ref(revision.parent['remote_startup_binding'])
        binding=dict(schema=1,node_name=REMOTE,master_uri=old_binding['master_uri'],
            parameters=revision.config['remote_parameters'],allowed_backends=old_binding['allowed_backends'],
            client_contract_source=old_binding['client_contract_source'],node_identity={k:v for k,v in remote.items() if k!='uri'},
            evidence=[revision.reference,construction_ref,ref(revision.output/'source_start.json'),ref(prefix_path)],
            construction_evidence_review_required=True,ready_health_not_implied_by_startup_binding=True,
            construction_provenance='R27 single remote unit; exact original entry/parameters, tested source preload')
        binding_ref=revision.save(revision.output/'remote_startup_binding.json',binding)
        ready=revision.health.evaluate(revision.health.load_binding(Path(binding_ref['path']),binding_ref['sha256']),revision.output)
        require(ready['passed'] is True,'REMOTE_DEPENDENCY_NOT_READY')
        health_ref=revision.save(revision.output/'remote_readiness.json',ready)
        revision.verify_files()
        require(revision.alias_snapshot(construction_ref)==before_aliases,'construction changed during reload')
        revision.graph(master,rospy.get_name(),remote,service,expected_aliases=before_aliases)
        after=wait_manual(revision,monitor,rospy,before['epoch_ns'],before)
        report.update(passed=True,nodes=nodes,auxiliary_nodes=aliases,remote_service=service,before=before,state=after,
            parent_acceptance=PARENT_ACCEPTANCE,parent_shutdown=stopped_ref,
            selection=selection,construction=construction_ref,remote_startup_binding=binding_ref,
            source_startup_receipt=ref(revision.output/'source_start.json'),
            remote_dependency=health_ref,retained_node_count=16,full_restart=False,
            remote_graph=before_remote_graph,reset_ack=acknowledgement)
    except BaseException as exc:report['error']=repr(exc);raise
    finally:
        if private is not None and private.exists():private.unlink()
        monitor.close();report['finished_ns']=time.time_ns();revision.save(revision.output/'reload_result.json',report)
    print(json.dumps(dict(passed=True,result=ref(revision.output/'reload_result.json')),ensure_ascii=False))


def accept(args):
    revision,review_ref=load_live(args)
    reload_ref=dict(path=str(args.reload.resolve()),sha256=args.reload_sha256);deployed=read_ref(reload_ref)
    validate_reload_chain(reload_ref,deployed,revision.reference,revision.output,review_ref)
    read_ref(deployed['source_startup_receipt'])
    aliases_expected=revision.alias_snapshot(deployed['construction'])
    require(aliases_expected==deployed['auxiliary_nodes'],'reload auxiliary snapshot differs from construction')
    out=revision.output/'acceptance';require(not out.exists(),'new acceptance output required');out.mkdir()
    os.environ.update(revision.storage.environment(out/'observer_ros',session=revision.session))
    import rospy
    import rosgraph
    from controller_manager_msgs.srv import ListControllers
    from alicia_d_driver.msg import PhysicalReferenceState, PhysicalReferenceCommand
    from alicia_d_driver.srv import SetPhysicalReference, SetPhysicalReferenceRequest
    rospy.init_node('r27_remote_only_acceptance_r4',anonymous=True,disable_rosout=True)
    master=rosgraph.Master(rospy.get_name());monitor=revision.stack.Monitor(revision.config)
    observer=revision.typed.Observer(rospy)
    report=dict(schema=1,kind='r27_remote_only_acceptance_r4',passed=False,revision=revision.reference,reload=reload_ref,
        parent_acceptance=PARENT_ACCEPTANCE,parent_shutdown=deployed['parent_shutdown'],
        execution_review=review_ref,started_ns=time.time_ns(),manual_only=True,automatic_grasp_started=False,
        full_restart=False,source_manifest=revision.data['release'],contract=revision.parent['contract'],
        preserved_typed_assets=revision.preserved_typed_evidence,**NO_ACTION)
    try:
        revision.verify_files();revision.live_policy(master)
        remote=deployed['nodes'][REMOTE];service=deployed['remote_service']
        nodes,aliases=revision.graph(master,rospy.get_name(),remote,service,expected_aliases=aliases_expected)
        require(nodes==deployed['nodes'] and aliases==deployed['auxiliary_nodes'],'reload instance changed')
        require(remote_graph(master)==deployed['remote_graph'],'accepted remote topic/service graph changed')
        for kind in (PhysicalReferenceState,PhysicalReferenceCommand,SetPhysicalReference):
            require(kind._md5sum==revision.assets['type_md5'][kind._type],'generated type MD5 differs')
            module_path=Path(sys.modules[kind.__module__].__file__).resolve()
            require(sha(module_path)==revision.assets['current_sha256'].get(str(module_path)),'generated binding path differs')
        for key,value in revision.assets['required_parameters'].items():
            require(rospy.get_param(key) is value and revision.config['parameters'].get(key) is value,'typed policy changed')
        binary_before=revision.typed.runtime_binaries(dict(nodes=nodes),revision.assets)
        graph_before=revision.typed.graph_bindings(master,revision.assets,probe=True)
        deadline=time.monotonic()+12;error=None
        while time.monotonic()<deadline:
            try:
                manual=wait_manual(revision,monitor,rospy,deployed['state']['epoch_ns'],deployed['state'])
                ready=revision.typed.observed_ready(observer,manual['epoch_ns'],rospy,revision.assets);break
            except RuntimeError as exc:error=str(exc);time.sleep(.02)
        else:raise RuntimeError('typed_not_ready without corrective action: '+str(error))
        query_states={}
        for (name,_owner),key in zip(revision.typed.SERVICES.items(),('driver','hardware')):
            request=SetPhysicalReferenceRequest();request.operation=request.QUERY
            response=revision.typed.bounded_query(rospy.ServiceProxy(name,SetPhysicalReference),request)
            require(response.success is True and response.code=='LEGACY','typed QUERY is not initial LEGACY')
            query_states[key]=response.state
        query_wait={}
        queried=revision.typed.wait_for_fixed_query_sources(observer,manual['epoch_ns'],rospy,revision.assets,query_states,query_wait)
        controllers=rospy.ServiceProxy('/controller_manager/list_controllers',ListControllers)().controller
        states={c.name:c.state for c in controllers}
        require(all(states.get(n)=='running' for n in ('alicia_controller','hand_controller')),'controller not running')
        gui=revision.stack.gui_windows(nodes['/alicia_supervisor_gui']['pid'])
        simulation=revision.stack.load(revision.stack.SIM)
        simulation.UNIT=revision.startup['components']['simulation']['unit']
        simulation.wait_ready(timeout=40.)
        require(simulation.service_snapshot()==revision.parent['simulation'],'paired simulation readiness/instance differs')
        binding=revision.health.load_binding(Path(deployed['remote_startup_binding']['path']),deployed['remote_startup_binding']['sha256'])
        health=revision.health.evaluate(binding,out);require(health['passed'] is True,'REMOTE_DEPENDENCY_NOT_READY')
        health_ref=revision.save(out/'remote_readiness.json',health)
        revision.verify_files();revision.live_policy(master)
        require(revision.alias_snapshot(deployed['construction'])==aliases_expected,'construction changed during acceptance')
        require(revision.graph(master,rospy.get_name(),remote,service,expected_aliases=aliases_expected)==(nodes,aliases),
                'runtime changed during acceptance')
        final=wait_manual(revision,monitor,rospy,manual['epoch_ns'],manual)
        final_typed=revision.typed.observed_ready(observer,final['epoch_ns'],rospy,revision.assets)
        graph_after=revision.typed.graph_bindings(master,revision.assets)
        require(graph_after['topics']==graph_before['topics'] and all(graph_after['services'][n]['uri']==r['uri']
                for n,r in graph_before['services'].items()),'typed service/topic ownership changed')
        require(revision.typed.runtime_binaries(dict(nodes=nodes),revision.assets)==binary_before,'runtime binary mapping changed')
        require(0<=time.time_ns()-health['finished_ns']<30_000_000_000,'remote health expired during typed acceptance')
        report.update(passed=True,nodes=nodes,auxiliary_nodes=aliases,remote_service=service,state=final,
            core_unit=revision.parent['core_unit'],core_service=revision.parent['core_service'],
            master_identity=revision.parent['master_identity'],simulation=revision.parent['simulation'],
            remote_startup_binding=deployed['remote_startup_binding'],remote_dependency=health_ref,
            source_startup_receipt=deployed['source_startup_receipt'],
            construction=deployed['construction'],
            all_configured_nodes_started=True,source_cache_process_binding=True,memory_attestation=False,
            typed_initial_contract_ready=True,physical_or_geometric_execution_ready=False,
            observed_initial_contract=ready,post_query_initial_contract=queried,query_receipt_source_wait=query_wait,
            final_typed_initial_contract=final_typed,typed_graph_before=graph_before,typed_graph_after=graph_after,
            binaries=binary_before,controllers=states,gui_windows=gui,retained_node_count=16,
            verification=revision.stack.current_release_evidence(revision.release))
    except BaseException as exc:report['error']=repr(exc);raise
    finally:
        observer.close();monitor.close();report['finished_ns']=time.time_ns();revision.save(out/'result.json',report)
    print(json.dumps(dict(passed=True,result=ref(out/'result.json')),ensure_ascii=False))


class AttemptContext:
    """Original Session interface with an explicitly checked remote-only contract."""
    def __init__(self,revision,acceptance_ref,instruction_ref,execution_review_ref,recorder):
        self.rev=revision;self.acceptance_ref=acceptance_ref;self.instruction_ref=instruction_ref
        self.acceptance=read_ref(acceptance_ref);self.instruction=read_ref(instruction_ref)
        self.recorder=recorder
        revision.entry.validate_instruction(self.instruction);revision.entry.validate_acceptance(self.acceptance)
        reload_ref=ref(revision.output/'reload_result.json')
        validate_acceptance_chain(acceptance_ref,self.acceptance,revision.reference,revision.output,
            revision.data['release'],revision.parent['contract'],execution_review_ref,reload_ref,read_ref(reload_ref))
        require(revision.alias_snapshot(self.acceptance['construction'])==self.acceptance['auxiliary_nodes'],
                'accepted auxiliary snapshot differs from construction')
        require(self.instruction.get('maximum_attempts')==1 and self.instruction.get('automatic_retry') is False,
                'one separately recorded attempt instruction required')
        self.contract=revision.contract;self.config=revision.config;self.stack=revision.stack
        self.release_ref=revision.data['release'];self.release=revision.release;self.revision_ref=None
        for attribute,path in (('boundary','tools/grasp_attempt_boundary.py'),('finish','tools/finish_returned_grasp.py'),
            ('idle','.runtime/restore_gen62_after_handoff_20261022.py'),('health','.runtime/verify_active_remote_readiness_20261020.py'),
            ('warm','.runtime/prewarm_mode_grasp_runner_20261012.py')):
            setattr(self,attribute,load_source(path,revision.sources))
        self.remote_ref=self.acceptance['remote_startup_binding']
        self.remote=self.health.load_binding(Path(self.remote_ref['path']),self.remote_ref['sha256'])
        require(self.remote['node_identity']=={k:v for k,v in self.acceptance['nodes'][REMOTE].items() if k!='uri'}
                and self.remote['parameters']==self.config['remote_parameters'],'accepted remote binding differs')

    def validate_contract(self,manual_return=False):
        self.rev.verify_files(manual_return=manual_return)
        require(read_ref(self.acceptance_ref)==self.acceptance and read_ref(self.instruction_ref)==self.instruction,
                'attempt inputs changed')
        read_ref(self.acceptance['source_startup_receipt'])
        require(self.rev.alias_snapshot(self.acceptance['construction'])==self.acceptance['auxiliary_nodes'],
                'attempt auxiliary construction changed')
        return self.rev.session,self.config,self.release

    def recheck(self,master,caller,manual_return=False):
        self.validate_contract(manual_return=manual_return)
        if manual_return:
            # Preserve the original Session cleanup domain: an unavailable
            # compute node cannot by itself prevent a safe manual return.
            require(self.stack.properties(self.acceptance['core_unit']) == self.acceptance['core_service']
                    and self.stack.process(master.getPid()) == self.acceptance['master_identity'],
                    'accepted core/master changed during manual return')
            for name in sorted(set(self.idle.OWNERS.values()) | {'/motion_gateway'}):
                require(self.rev.node(master,name,caller) == self.acceptance['nodes'][name],
                        'manual-return owner changed: '+name)
            return
        self.rev.simulation_policy(master)
        nodes,aliases=self.rev.graph(master,caller,self.acceptance['nodes'][REMOTE],self.acceptance['remote_service'],
            expected_aliases=self.acceptance['auxiliary_nodes'],recorder=self.recorder)
        require(nodes==self.acceptance['nodes'] and aliases==self.acceptance['auxiliary_nodes'],'accepted runtime changed')
        if not manual_return:self.health.check_binding(self.remote,self.health.observe(self.remote))


def attempt(args):
    revision,_review=load_live(args)
    record=Path(os.environ['GRASP_RECORD_DIR']).absolute()
    revision.storage.validate(record,subtree=revision.storage.archive_root,expected=revision.binding)
    require(record.is_dir() and record.parent==revision.storage.archive_root,'current recorder directory required')
    ready=json.loads((record/'recorder_ready.json').read_bytes())
    require(ready['record_dir']==str(record) and type(ready['pid']) is int
            and (Path('/proc')/str(ready['pid'])).is_dir(),'current recorder not ready')
    require(not (record/'authorization.json').exists(),'record already has an attempt authorization')
    manifest=json.loads((record/'manifest.json').read_bytes());metadata=manifest['metadata']
    require(metadata['launcher']=='run_recorded_grasp.py' and metadata['recording_duration_seconds']==600.
            and metadata['owner_pid']==os.getppid() and metadata['storage']==revision.binding,
            'original 600-second SSD recorder parent required')
    instruction_ref=dict(path=str(args.instruction.resolve()),sha256=args.instruction_sha256)
    recorder=RecorderBinding(revision,record,instruction_ref)
    revision.entry.configure_attempt_environment(record)
    context=AttemptContext(revision,dict(path=str(args.acceptance.resolve()),sha256=args.acceptance_sha256),
        instruction_ref,_review,recorder)
    revision.save(revision.output/('attempt_claim_'+args.instruction_sha256+'.json'),
        dict(instruction=context.instruction_ref,acceptance=context.acceptance_ref,record=str(record),
             recorder_ready=recorder.ready_ref,recording_manifest=recorder.manifest_ref,
             pid=os.getpid(),claimed_ns=time.time_ns(),automatic_retry=False))
    session=revision.entry.Session(context,record)
    session.report.update(runtime_revision=revision.reference,runtime_nodes_changed=True,changed_nodes=[REMOTE],
        full_restart=False,current_full_regression_passed=revision.release['full_regression_passed'],
        preserved_typed_assets=revision.preserved_typed_evidence)
    try:code=revision.entry.attempt_lifecycle(session)
    finally:
        session.report['recorder_binding']=recorder.reference
        closed=session.close()
    return code if closed else (code or 1)


def parser():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    prep=sub.add_parser('prepare')
    for key in ('baseline','typed','regression','source-review','instruction'):
        prep.add_argument('--'+key,type=Path,required=True);prep.add_argument('--'+key+'-sha256',required=True)
    prep.add_argument('--output',type=Path,required=True);prep.add_argument('--runtime-child',required=True)
    for action in ('reload','accept','attempt'):
        q=sub.add_parser(action)
        for key in ('revision','execution-review'):
            q.add_argument('--'+key,type=Path,required=True);q.add_argument('--'+key+'-sha256',required=True)
        keys=('reload',) if action=='accept' else (('acceptance','instruction') if action=='attempt' else ())
        for key in keys:q.add_argument('--'+key,type=Path,required=True);q.add_argument('--'+key+'-sha256',required=True)
    q=sub.add_parser('child');q.add_argument('--private',type=Path,required=True);q.add_argument('--private-sha256',required=True)
    return p


def main(argv=None):
    args,unknown=parser().parse_known_args(argv)
    require(not unknown or (args.action=='child' and all(x.startswith('__name:=') for x in unknown)), 'unexpected CLI arguments')
    return {'prepare':prepare,'reload':reload_remote,'accept':accept,'attempt':attempt,'child':child}[args.action](args)


if __name__=='__main__':
    sys.exit(main())
