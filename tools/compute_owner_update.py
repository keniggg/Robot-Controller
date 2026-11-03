"""Data-bound update of the three compute owners; import and plan are offline.

The sealed baseline is historical evidence, not current readiness. Live actions
retain the existing graph/SDK/configuration/remote/installation primitives. No
mode, motion, temperature, disable or automatic-retry operation is provided.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import traceback
from types import FunctionType, ModuleType, SimpleNamespace

from runtime_evidence import ReleaseEvidence, normalize_sha256

TARGETS = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
P = 'src/alicia_grasp_modes/'
ENTRIES = dict(zip(TARGETS, (P+'scripts/mode_aware_grasp_task.py',
    P+'scripts/mode_aware_remote_grasp6d.py', P+'scripts/unknown_tabletop_perception.py')))
ROLES = ('runtime', 'base', 'identity', 'verifier', 'old', 'checker', 'installer', 'preloader')
REFS = ('baseline_release', 'baseline_acceptance', 'baseline_reload', 'baseline_bundle',
    'baseline_construction', 'post_attempt', 'baseline_full', 'targeted', 'review',
    'adoption', 'startup_configuration', 'startup', 'core_reference', 'gateway_service_design')
EXPERIMENT_ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def strict_json(raw):
    def pairs(items):
        value = {}
        for key, item in items:
            require(key not in value, 'duplicate JSON key: '+key)
            value[key] = item
        return value
    def invalid(value):
        raise ValueError('nonfinite JSON value: '+value)
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid)


def reference(value):
    require(type(value) is dict and set(value) == {'path', 'sha256'}, 'exact path/SHA reference required')
    path = Path(value['path'])
    require(path.is_absolute(), 'absolute reference path required')
    return next(iter(normalize_sha256({path: value['sha256']}).items()))


def relative_map(value):
    require(type(value) is dict and bool(value), 'nonempty source map required')
    for name, digest in value.items():
        path = Path(name)
        require(type(name) is str and not path.is_absolute() and '..' not in path.parts
            and str(path) == name and re.fullmatch('[0-9a-f]{64}', digest) is not None,
            'unsafe source mapping')
    return dict(value)


def selection(value):
    require(type(value) is dict and set(value) == {'mode', 'strategy', 'generation', 'stamp_ns'}
        and value['mode'] == 'unknown' and value['strategy'] == 'observe_return'
        and type(value['generation']) is int and value['generation'] >= 0
        and type(value['stamp_ns']) is int and value['stamp_ns'] > 0, 'invalid selection identity')
    return value


def rebind(module, **updates):
    """Use unchanged primitive code with this transaction's data namespace."""
    namespace = dict(module.__dict__, **updates)
    for name, value in module.__dict__.items():
        if isinstance(value, FunctionType) and value.__globals__ is module.__dict__:
            function = FunctionType(value.__code__, namespace, value.__name__, value.__defaults__, value.__closure__)
            function.__kwdefaults__ = value.__kwdefaults__
            namespace[name] = function
    namespace.update(updates)
    return SimpleNamespace(**namespace)


class UpdateTransaction:
    """One explicit update invocation; cached bytes are historical only."""
    def __init__(self, path, digest):
        self.path, self.digest = reference(dict(path=str(Path(path).resolve()), sha256=digest))
        raw = self.path.read_bytes()
        require(hashlib.sha256(raw).hexdigest() == digest, 'update contract changed')
        self.data = d = strict_json(raw)
        require(d['schema'] == 1 and d['kind'] == 'three_compute_owner_update'
            and type(d['release_label']) is str and bool(d['release_label']), 'unsupported update contract')
        self.workspace = Path(d['workspace_root']).resolve()
        self.candidate = Path(d['candidate_root']).resolve()
        self.mount = Path(d['storage_root']).resolve()
        self.output_root = Path(d['output_root']).resolve()
        require(self.workspace.is_dir() and self.candidate.is_dir()
            and self.candidate != self.workspace and self.mount in self.output_root.parents,
            'workspace/candidate/output boundaries differ')
        require(type(d['reference']['epoch_ns']) is int and d['reference']['epoch_ns'] > 0,
            'positive reference epoch required')
        self.expected_selection = selection(d['reference']['selection'])
        self.expected_epoch = d['reference']['epoch_ns']
        require(set(d['refs']) == set(REFS) and set(d['modules']) == set(ROLES), 'explicit evidence/module roles required')
        self.current = normalize_sha256(d['current_sha256'], {self.path: digest})
        self.history = normalize_sha256(d['historical_sha256'])
        for name in ('authorization', 'control', 'archive_config', 'storage_control', 'starter'):
            p, h = reference(d[name]); require(self.current.get(p) == h, name+' must be a current input')
        for value in list(d['modules'].values())+d['module_dependencies']:
            p, h = reference(value); require(self.current.get(p) == h, 'executing module must be current')
        for p in (Path(__file__).resolve(), Path(__file__).with_name('runtime_evidence.py').resolve()):
            require(self.current.get(p) == sha(p), 'generic entry/component current pin missing')
        require(all(self.workspace not in p.parents and self.candidate not in p.parents
            and p != self.workspace and p != self.candidate for p in self.history),
            'workspace/candidate code cannot be historical evidence')
        self.evidence = ReleaseEvidence(self.history, current_paths=self.current)
        self.transaction = self.evidence.transaction()
        self.values = {name: self.read_ref(value) for name, value in d['refs'].items()}
        self.validate_values()
        self.phase_sources = self.baseline_sources
        require(not set(self.history).intersection(self.current_map()), 'live source/configuration was classified historical')
        self.modules = None

    def close(self):
        ctx = getattr(self, '_live_ctx', {})
        window = ctx.pop('_idle_window', None)
        try:
            if window is not None:
                window.close()
        finally:
            self.transaction.close()

    def __enter__(self):
        return self

    def __exit__(self, _kind, error, _trace):
        try:
            self.close()
        except BaseException as cleanup:
            if error is None:
                raise
            error.compute_owner_cleanup_error = repr(cleanup)

    def read_ref(self, value):
        path, digest = reference(value)
        require(self.history.get(path) == digest, 'sealed reference not explicitly historical: '+str(path))
        return strict_json(self.transaction.read_bytes(path))

    def read_path(self, path):
        path = Path(path).resolve()
        if path in self.history:
            return strict_json(self.transaction.read_bytes(path))
        require(path in self.current, 'undeclared current JSON input: '+str(path))
        raw = path.read_bytes()
        require(hashlib.sha256(raw).hexdigest() == self.current[path], 'current JSON input changed')
        return strict_json(raw)

    def validate_values(self):
        d, v = self.data, self.values
        release, accepted, prior, bundle, construction, post = [v[k] for k in
            ('baseline_release', 'baseline_acceptance', 'baseline_reload', 'baseline_bundle', 'baseline_construction', 'post_attempt')]
        refs = d['refs']
        require(release['passed'] is True and release['installed'] is True
            and release['acceptance'] == refs['baseline_acceptance']
            and accepted['deployment_passed'] is True and prior['passed'] is True,
            'baseline release was not accepted')
        require(dict(path=accepted['reload'], sha256=accepted['reload_sha256']) == refs['baseline_reload']
            and dict(path=accepted['bundle'], sha256=accepted['bundle_sha256']) == refs['baseline_bundle']
            and prior['bundle'] == accepted['bundle'] and prior['bundle_sha256'] == accepted['bundle_sha256']
            and dict(path=prior['construction'], sha256=prior['construction_sha256']) == refs['baseline_construction']
            and construction['bundle'] == accepted['bundle'] and construction['bundle_sha256'] == accepted['bundle_sha256'],
            'accepted transition/construction closure differs')
        self.baseline_sources = relative_map(release['source_sha256'])
        require(self.baseline_sources == accepted['source_sha256'] == bundle['source_sha256'] == construction['source_sha256'],
            'accepted source maps differ')
        self.nodes = accepted['full_runtime_nodes']
        self.preserved = {n:r for n,r in self.nodes.items() if n not in TARGETS}
        require(len(self.nodes) == 17 and len(self.preserved) == 14
            and set(accepted['targets']) == set(prior['targets']) == set(prior['units']) == set(TARGETS)
            and accepted['targets'] == prior['targets'] == {n:self.nodes[n] for n in TARGETS}
            and accepted['preserved'] == bundle['preserved'] == self.preserved
            and accepted['before']['node_identities'] == accepted['after']['node_identities'] == self.nodes
            and prior['after']['node_identities'] == self.nodes, 'three/14 accepted identity partition differs')
        for row in self.nodes.values():
            require(set(row) == {'pid', 'parent_pid', 'start_ticks'} and type(row['pid']) is int
                and type(row['parent_pid']) is int and row['pid'] > 1 and row['parent_pid'] > 1
                and type(row['start_ticks']) is str and row['start_ticks'].isdigit(), 'invalid process identity')
        require(post['passed'] is True and post['accepted_processes_unchanged'] is True
            and post['source_manifest'] == refs['baseline_release']['path']
            and post['source_manifest_sha256'] == refs['baseline_release']['sha256']
            and post['deployment'] == refs['baseline_acceptance']['path']
            and post['deployment_sha256'] == refs['baseline_acceptance']['sha256']
            and post['full_runtime_nodes_before'] == post['full_runtime_nodes_after'] == self.nodes
            and post['epoch_ns'] == self.expected_epoch and post['selection'] == self.expected_selection
            and post['prior_accepted_selection'] == accepted['selection']
            and post['gui_own_manual_ack'] is True and post['task_active'] is False and post['actions_active'] is False
            and post['sample_count'] == 20 and post['actuation_status'].startswith('CONFIRMED:'),
            'closed post-attempt reference differs')
        exited, authorization, record = [self.read_ref(post[k]) for k in
            ('original_runner_exit', 'selection_authorization', 'closed_record_manifest')]
        require(exited['selection'] == authorization['selection'] == self.expected_selection
            and authorization['previous_selection'] == accepted['selection']
            and authorization['operator_authorized_autonomous_grasp'] is True
            and record['state'] == 'complete' and record['recorder_closed'] is True
            and record['record_id'] == Path(post['record_dir']).name, 'returned closed authorized record differs')
        require(self.expected_selection['generation'] == accepted['selection']['generation']+1
            and self.expected_selection['stamp_ns'] > accepted['selection']['stamp_ns'],
            'post-attempt must be the exact single authorized selection transition')
        full = v['baseline_full']
        require(dict(path=release['actual_regression'], sha256=release['actual_regression_sha256']) == refs['baseline_full']
            and full['passed'] is True and full['full_regression_passed'] is True
            and full['source_sha256'] == self.baseline_sources and full['summary'] == release['test_summary'],
            'historical full baseline differs')
        change = d['source_update']; candidate = relative_map(change['candidate_sha256'])
        self.production, self.tests = set(change['production']), set(change['tests'])
        require(self.production and not self.production & self.tests and set(candidate) == self.production | self.tests
            and all('/tests/' not in p for p in self.production)
            and all('/tests/' in p for p in self.tests)
            and self.production <= set(self.baseline_sources)
            and all(candidate[p] != self.baseline_sources.get(p) for p in candidate), 'unreviewed source delta')
        self.candidate_sources = candidate
        self.sources = dict(self.baseline_sources, **candidate)
        focused, review, adoption = v['targeted'], v['review'], v['adoption']
        require(focused['passed'] is True and focused['full_regression_passed'] is False
            and focused['exact_case_inventory'] is True and not focused['duplicate_cases'] and not focused['omitted_cases']
            and focused['production_sha256'] == {p:candidate[p] for p in self.production}
            and focused['current_test_sha256'] == {p:candidate[p] for p in self.tests}, 'focused verification scope differs')
        cases = focused['case_results']; identities = [tuple(row['identity']) for row in cases]
        require(len(cases) == len(set(identities)) == focused['total_distinct_case_count']
            and all(row['passed'] is True for row in cases), 'targeted case omission/failure/duplication')
        expected_review = {str(self.candidate/p):candidate[p] for p in self.production}
        require(review['passed'] is True and not review['blocking_items']
            and review['candidate_source_sha256'] == expected_review, 'independent review bytes differ')
        expected_impact = {p:(self.baseline_sources[p],candidate[p]) for p in self.production}
        require({r['path']:(r['baseline_sha256'],r['candidate_sha256']) for r in adoption['source_map']} == expected_impact
            and set(adoption['affected_processes']) == set(TARGETS)
            and set(adoption['preserved_instance_names']) == set(self.preserved)
            and adoption['preserved_instance_count'] == 14
            and adoption['gateway_compatibility']['require_gateway_restart_for_this_patch'] is False
            and adoption['gateway_compatibility']['memory_attestation'] is False, 'adoption/retained gateway scope differs')
        require(bundle['entries'] == ENTRIES and set(construction['processes']) == set(TARGETS), 'original entry scope differs')
        config = v['startup_configuration']
        require(all(config['nodes'][n]['respawn'] is False and config['nodes'][n]['required'] is False for n in TARGETS),
            'original launch policy differs')
        require(construction['remote_parameters'] == config['remote_parameters'], 'accepted remote configuration differs')
        self.configuration_sources = normalize_sha256(config['launch_source_sha256'],
            {config['client_contract_source']['path']:config['client_contract_source']['sha256']})
        require(dict(path=v['startup']['startup_configuration'],sha256=v['startup']['startup_configuration_sha256'])
            == refs['startup_configuration'], 'original startup configuration binding differs')
        core = v['core_reference']
        require(core['core_service'] == accepted['after']['core_service']
            and core['simulation_service'] == accepted['after']['simulation_service'], 'retained core/simulation binding differs')
        service_names = v['gateway_service_design']['constructor_review']['registered_service_names']
        require(len(service_names) == len(set(service_names)) == 31, 'gateway service inventory differs')
        self.load_resolution = relative_map(bundle['load_resolution_sha256'])
        self.expected_control = accepted['control_snapshot']
        for event in d.get('control_events', []):
            next_control, request = self.read_ref(event['snapshot']), self.read_ref(event['request'])
            require(self.expected_control['state'] not in ('revoked', 'completed')
                and next_control['revision'] == self.expected_control['revision']+1
                and next_control['authorization_id'] == self.expected_control['authorization_id']
                and request['source'] == 'actual_user_message' and bool(request['text']), 'invalid explicit control event')
            if next_control['state'] == 'active':
                require(self.expected_control['state'] == 'paused'
                    and next_control['resume_instruction'] == event['request']
                    and next_control['resume_user_message'] == request['text']
                    and request['new_hands_clear_confirmation_claimed'] is False, 'resume event binding differs')
            else:
                require(next_control['state'] in ('paused', 'revoked', 'completed'), 'invalid control transition')
            self.expected_control = next_control
        # Finite result inputs are verified once; code/config inputs stay current.
        for result in (focused, review):
            for p,h in result.get('input_sha256', {}).items():
                path = Path(p).resolve()
                known = self.current.get(path, self.history.get(path))
                if self.candidate in path.parents:
                    known = candidate.get(str(path.relative_to(self.candidate)))
                require(known == h, 'verification input not explicitly classified: '+p)

    def current_map(self, sources=None):
        sources = self.phase_sources if sources is None else sources
        return normalize_sha256(self.current,
            self.configuration_sources,
            {self.workspace/p:h for p,h in sources.items()},
            {self.workspace/p:h for p,h in self.load_resolution.items()},
            {self.candidate/p:h for p,h in self.candidate_sources.items()})

    def recheck(self, _ctx=None, sources=None):
        json_paths = {Path(self.data[k]['path']).resolve() for k in
            ('authorization', 'control', 'archive_config', 'storage_control')}
        current_json = {}
        for path, digest in self.current_map(sources).items():
            require(path.is_file() and not path.is_symlink(), 'current input is not a regular source: '+str(path))
            raw = path.read_bytes()
            require(hashlib.sha256(raw).hexdigest() == digest, 'current input changed: '+str(path))
            if path in json_paths:
                current_json[path] = strict_json(raw)
        self.transaction.verify_all()
        auth = current_json[Path(self.data['authorization']['path']).resolve()]
        control = current_json[Path(self.data['control']['path']).resolve()]
        require(auth['version'] == 1 and auth['scope'] == 'observe_return_until_physical_grasp_success'
            and auth['confirmation_source']['source'] == 'actual_user_message'
            and control == self.expected_control and control['state'] == 'active'
            and control['authorization_id'] == auth['authorization_id'], 'current authorization/control changed')
        archive = current_json[Path(self.data['archive_config']['path']).resolve()]
        policy = current_json[Path(self.data['storage_control']['path']).resolve()]
        require(archive['upload_enabled'] is False and archive['delete_enabled'] is False
            and policy['archive_policy'] == 'local_only', 'storage policy changed')
        require(self.mount.is_mount() and not os.statvfs(self.mount).f_flag & os.ST_RDONLY,
            'writable external storage unavailable')
        stat = os.statvfs(self.mount)
        require(stat.f_bavail*stat.f_frsize >= archive['min_free_bytes'], 'external reserve unavailable')
        return control

    def storage(self, path):
        path = Path(path).resolve()
        require(self.output_root in path.parents and self.mount.is_mount(), 'external update output required')
        ancestor = path.parent
        while not ancestor.exists():
            ancestor = ancestor.parent
        require(ancestor.stat().st_dev == self.mount.stat().st_dev
            and not os.statvfs(self.mount).f_flag & os.ST_RDONLY, 'external output unavailable; no fallback')

    def save(self, path, value):
        self.storage(path)
        raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
        with Path(path).open('xb') as stream:
            stream.write(raw); stream.flush(); os.fsync(stream.fileno())
        require(Path(path).read_bytes() == raw, 'receipt readback differs')

    def load_modules(self):
        """Execute verified current helper source; do not import helper pyc."""
        loaded = {}
        for ref in self.data['module_dependencies']+[self.data['modules'][role] for role in ROLES]:
            path, digest = reference(ref)
            if path in loaded:
                continue
            raw = path.read_bytes(); require(hashlib.sha256(raw).hexdigest() == digest, 'executing helper changed')
            name = path.stem
            require(name not in sys.modules or getattr(sys.modules[name], '__file__', None) == str(path),
                'helper import name collision')
            module = ModuleType(name); module.__file__ = str(path)
            sys.modules[name] = module
            try:
                exec(compile(raw, str(path), 'exec'), module.__dict__)
            except BaseException:
                sys.modules.pop(name, None); raise
            loaded[path] = module
        self.modules = {role:loaded[reference(ref)[0]] for role,ref in self.data['modules'].items()}
        return self.modules

    def live_context(self, for_update=False):
        require(not self.data.get('draft_pending'), 'draft contract cannot invoke live operations')
        self.recheck()
        modules = self.load_modules(); v = self.values
        runtime = rebind(modules['runtime'], W=self.workspace, C=self.candidate,
            PRODUCTION=tuple(sorted(self.production)), TARGETS=TARGETS, P=P,
            EPOCH=self.expected_epoch, SELECTION=self.expected_selection,
            read=self.read_runtime, save=self.save, storage=self.storage)
        base = rebind(modules['base'], W=self.workspace, recheck=self.recheck,
            read=self.read_path, save=self.save, storage=self.storage,
            DESIGN=Path(self.data['refs']['gateway_service_design']['path']),
            DESIGN_SHA=self.data['refs']['gateway_service_design']['sha256'],
            bind=lambda ctx,path,digest:self.read_ref(dict(path=str(path),sha256=digest)))
        old = SimpleNamespace(**modules['old'].__dict__)
        old.CONTROL = Path(self.data['control']['path'])
        ctx = dict(old=old, v=modules['verifier'], runtime59=modules['identity'],
            checker=modules['checker'], previous=v['core_reference'], config=v['startup_configuration'],
            startup=v['startup'], nodes=self.nodes, preserved=self.preserved,
            expected_gateway=self.nodes['/motion_gateway'],
            prior_transition=v['baseline_reload'], prior_construction=v['baseline_construction'])
        require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == v['startup']['boot_id'], 'boot identity changed')
        binding_ref = v['baseline_reload']['remote_binding']
        self.read_ref(binding_ref)
        binding = modules['checker'].load_binding(Path(binding_ref['path']), binding_ref['sha256'])
        require(binding['parameters'] == ctx['config']['remote_parameters']
            and all(binding['node_identity'][k] == self.nodes['/remote_grasp6d_node'][k]
                for k in ('pid','parent_pid','start_ticks')), 'accepted remote instance/config binding differs')
        ctx['binding'] = binding
        ctx['recheck'] = self.recheck
        self._live_ctx = ctx
        if for_update:
            expected = {}
            for name in TARGETS:
                row = self.nodes[name]
                require(ctx['v'].process(row['pid']) == row, 'accepted compute identity changed')
                proof = v['baseline_acceptance']['source_process_proof'][name]
                receipt = self.read_ref(dict(path=proof['receipt'],sha256=proof['sha256']))
                proc = Path('/proc')/str(row['pid']); command = (proc/'cmdline').read_bytes()
                original = v['baseline_construction']['processes'][name]
                require(receipt['node_identity'] == dict(row,cmdline_sha256=hashlib.sha256(command).hexdigest())
                    and receipt['source_sha256'] == self.baseline_sources
                    and receipt['entry'] == original['entry'] == ENTRIES[name]
                    and receipt['run_path'] == original['run_path']
                    and Path(original['run_path']).is_file() and Path(original['original_sys_path0']).is_dir(),
                    'accepted source/start command identity changed')
                expected[name] = dict(identity=row, argv=command.rstrip(b'\0').decode().split('\0'),
                    cwd=original['cwd'], environment=original['environment'])
            ctx['expected_processes'] = expected
        return base, runtime, ctx

    def read_runtime(self, path):
        path = Path(path).resolve()
        if path in self.current or path in self.history:
            return self.read_path(path)
        require(self.output_root in path.parents and path.is_file() and not path.is_symlink(),
            'runtime receipt outside declared output')
        return strict_json(path.read_bytes())

    def checked_master(self, base, ctx, master, caller):
        declarations = self.data.get('declared_readonly_observers', [])
        if not declarations:
            return master
        path, digest = reference(self.data['observer_module'])
        require(self.current.get(path) == digest and sha(path) == digest, 'observer validator is not current')
        module = ModuleType('compute_update_observer_contract'); module.__file__ = str(path)
        raw=path.read_bytes(); require(hashlib.sha256(raw).hexdigest()==digest,'observer validator changed while reading')
        exec(compile(raw, str(path), 'exec'), module.__dict__)
        def sources(mapping):
            for p,h in normalize_sha256(mapping).items():
                require(self.current.get(p) == h and sha(p) == h, 'declared observer source changed')
            return True
        def identity(name):
            value=base.identity(ctx, master, caller, name)
            expected=self.data['readonly_observer_processes'][name]
            folder=Path('/proc')/str(value['pid'])
            argv=(folder/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
            require(argv==expected['argv'] and str((folder/'cwd').resolve())==expected['cwd'],
                'declared observer command/cwd changed')
            return value
        return module.checked_master(master, caller, declarations,
            identity, sources)

    def bundle(self):
        prior = self.values['baseline_bundle']
        return dict(kind='three_compute_owner_update', deployment_ready=True,
            contract=str(self.path), contract_sha256=self.digest,
            baseline_source_sha256=self.baseline_sources, source_sha256=self.sources,
            production=sorted(self.production), additions=sorted(self.tests),
            nodes=self.nodes, preserved=self.preserved, entries=ENTRIES,
            load_resolution_sha256=self.load_resolution,
            cached_parameter_cleanup_keys=prior['cached_parameter_cleanup_keys'],
            gateway_compatibility=self.values['adoption']['gateway_compatibility'])

    def summary(self):
        return dict(kind='three_compute_owner_update_plan', passed=True,
            release_label=self.data['release_label'], contract=str(self.path), contract_sha256=self.digest,
            source_sha256=self.sources, replaced_nodes=list(TARGETS), preserved_nodes=sorted(self.preserved),
            baseline_full=self.data['refs']['baseline_full'], baseline_full_scope='baseline release only',
            current_targeted=self.data['refs']['targeted'],
            current_targeted_cases=self.values['targeted']['total_distinct_case_count'],
            candidate_full_regression_passed=False, live_readiness_proven=False,
            automatic_attempt_authorized=False, history_work=self.transaction.stats)


class IdleWindow:
    """Bounded callback state for one unchanged driver/reference, one process."""
    def __init__(self, driver, epoch):
        self.lock=threading.Lock();self.values={};self.history=[];self.subscribers=[]
        self.driver=deepcopy(driver);self.epoch=epoch;self.invalidated=False
        self.sampling=False

    def receive(self, key, msg, received):
        with self.lock:
            self.values[key]=(msg,received)
            if key in ('joint','epoch') and msg._connection_header.get('callerid')!='/alicia_d_driver_node':
                self.invalidated=True
            if key=='epoch' and msg.stamp.to_nsec()!=self.epoch:
                self.invalidated=True
            if key=='joint':
                self.history.append(dict(stamp_ns=msg.header.stamp.to_nsec(),received_ns=received,
                    frame_id=msg.header.frame_id,names=list(msg.name),positions=list(msg.position),
                    callerid=msg._connection_header.get('callerid')))
                self.history[:]=self.history[-80:]

    def require_reference(self, driver, epoch):
        require(not self.invalidated and self.driver==driver and self.epoch==epoch,
            'SDK window driver/epoch changed; new transaction required')

    def close(self):
        subscribers, self.subscribers = self.subscribers, []
        self.invalidated=True
        failures=[]
        for subscriber in subscribers:
            try:
                subscriber.unregister()
            except BaseException as error:
                failures.append(repr(error))
        require(not failures, 'SDK subscription cleanup failed: '+repr(failures))


def sample_idle(ctx, rospy, master, expected_selection, expected_epoch, task_absent=False):
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header, String, Bool
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.selection import parse_selection
    from controller_manager_msgs.srv import ListControllers
    topics = [('joint','/alicia_d/accepted_joint_states',JointState),('epoch','/alicia_d/control_reference_epoch',Header),
        ('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String),('gui','/gui/joint_direct_mode',Bool),
        ('actuation','/alicia_d/actuation_status',String),('motion','/alicia_d/motion_enabled',Bool),
        ('feedback','/alicia_d/feedback_ready',Bool),('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray)]
    owners = {'joint':'/alicia_d_driver_node','epoch':'/alicia_d_driver_node','task':'/grasp_task_node',
              'mode':'/grasp_mode_router','gui':'/alicia_supervisor_gui','actuation':'/alicia_d_driver_node',
              'motion':'/alicia_d_driver_node','feedback':'/alicia_d_driver_node',
              'arm':'/bessica_d_hw_interface','hand':'/bessica_d_hw_interface'}
    driver=ctx['preserved']['/alicia_d_driver_node']
    window=ctx.get('_idle_window')
    if window is None:
        window=IdleWindow(driver,expected_epoch);ctx['_idle_window']=window
        window.subscribers=[rospy.Subscriber(t,k,lambda m,key=n:window.receive(key,m,rospy.Time.now().to_nsec()),queue_size=40)
            for n,t,k in topics]
    window.require_reference(driver,expected_epoch)
    require(not window.sampling,'concurrent idle sampling is unsupported')
    window.sampling=True
    lock,values,history=window.lock,window.values,window.history
    if task_absent:
        topics=[row for row in topics if row[0]!='task']
    required={row[0] for row in topics}
    def receive(key,msg):
        window.receive(key,msg,rospy.Time.now().to_nsec())
    try:
        end = time.monotonic()+8.
        while time.monotonic() < end:
            with lock:
                window.require_reference(driver,expected_epoch)
                ready = required <= set(values) and len(history) >= 20
            if ready:
                break
            time.sleep(.025)
        services=dict(master.getSystemState()[2])
        require(services.get('/controller_manager/list_controllers')==['/bessica_d_hw_interface'], 'controller service ownership changed')
        publishers=dict(master.getSystemState()[0])
        require(all(publishers.get(topic)==['/bessica_d_hw_interface'] for topic in (
            '/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status')),
            'action status publisher ownership changed')
        rospy.wait_for_service('/controller_manager/list_controllers',timeout=5.)
        controllers={item.name:item.state for item in rospy.ServiceProxy('/controller_manager/list_controllers',ListControllers)().controller}
        require(all(controllers.get(name)=='running' for name in ('alicia_controller','hand_controller')), 'required controllers not running')
        # Current files can take time to verify. Keep receiving SDK feedback,
        # then validate the latest window and freshly fetched state below.
        ctl=ctx['recheck'](ctx)
        for key,topic,kind in topics:
            if key not in ('joint','arm','hand'):
                receive(key,rospy.wait_for_message(topic,kind,timeout=3.))
        with lock:
            window.require_reference(driver,expected_epoch)
            states = {k:v[0] for k,v in values.items() if k in required}; times = {k:v[1] for k,v in values.items() if k in required}
            samples=list(history[-20:])
        require(len(states) == len(topics) and len(samples) == 20, 'missing state or 20 unique SDK samples')
        now = rospy.Time.now().to_nsec(); q=states['joint']; epoch=states['epoch'].stamp.to_nsec()
        require(not rospy.is_shutdown() and all(0<=now-times[k]<1_000_000_000 for k in states), 'stale current state receipt')
        require(all(states[k]._connection_header.get('callerid')==owners[k] for k in states), 'state publisher owner changed')
        names=samples[-1]['names']; stamps=[s['stamp_ns'] for s in samples]
        require(len(set(names))==len(names) and {'Joint%d'%i for i in range(1,7)}<=set(names), 'invalid joint layout')
        require(len(set(stamps))==20 and all(b>a for a,b in zip(stamps,stamps[1:]))
                and 0<=now-stamps[-1]<200_000_000 and epoch>0, 'SDK arrivals not unique, ordered or fresh')
        require(all(s['frame_id']=='sdk_measured' and s['callerid']=='/alicia_d_driver_node' and s['names']==names
                    and len(s['positions'])==len(names) and all(math.isfinite(x) for x in s['positions'])
                    and epoch<=s['stamp_ns']<=s['received_ns'] and s['received_ns']-s['stamp_ns']<200_000_000
                    for s in samples), 'invalid SDK source, layout or per-message age')
        spans={n:max(s['positions'][i] for s in samples)-min(s['positions'][i] for s in samples) for i,n in enumerate(names)}
        require(all(x<=.004 for x in spans.values()), 'joint movement during idle sampling')
        reference=ctx.setdefault('idle_reference',dict(names=names,positions=samples[-1]['positions']))
        require(reference['names']==names and len(reference['positions'])==len(names), 'transition reference layout changed')
        maximum=max(abs(s['positions'][i]-reference['positions'][i]) for s in samples for i in range(len(names)))
        require(maximum<=.004, 'pose moved from transition reference')
        require((task_absent or not states['task'].active) and states['gui'].data is True and states['gui']._connection_header.get('latching')=='1' and master.getParam('/gui/joint_direct_mode') is True, 'not GUI-owned manual idle')
        router=json.loads(states['mode'].data)
        require(not any(router.get(k) for k in ('active','start_reserved','pending')), 'router active')
        require(all(0<=now-states[k].header.stamp.to_nsec()<=2_000_000_000 and not any(g.status in (0,1,6,7) for g in states[k].status_list)
                    for k in ('arm','hand')), 'active or stale action status')
        require(states['feedback'].data is True and states['motion'].data is True
                and states['actuation'].data.startswith('CONFIRMED:')
                and master.getParam('/grasp/require_actuation_confirmation') is True, 'measured actuation/feedback not ready')
        require(master.getParam('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
                and master.getParam('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.
                and master.getParam('/grasp/unknown_lift_after_close_enabled') is False
                and master.getParam('/grasp/unknown_observe_return_max_observations') == 1, 'temperature/lift/observation policy changed')
        selection=parse_selection(master.getParam('/grasp_mode/selection'))
        require(selection['generation']==expected_selection['generation'] and selection['mode']=='unknown' and selection['strategy']=='observe_return', 'selection differs')
        require(selection==expected_selection, 'selection changed during transition')
        require(epoch == expected_epoch, 'control reference epoch changed; fresh reference required')
        # Reusing a live window never turns an old snapshot into fresh feedback.
        with lock:
            window.require_reference(driver,expected_epoch)
        require(0<=rospy.Time.now().to_nsec()-stamps[-1]<200_000_000,
            'SDK snapshot expired during current input verification')
        return dict(recorded_ns=time.time_ns(),epoch_ns=epoch,selection=selection,sample_count=20,names=names,
            positions=samples[-1]['positions'],sample_position_spans=spans,sample_stable_limit=.004,
            maximum_joint_delta_rad=maximum,
            sample_stamps_ns=stamps,samples=samples,state_received_ns=times,
            latch_receipt_time_is_not_source_generation_time=True,manual_mode=True,gui_owning_manual_ack=True,
            task_active=False,task_state=('PROCESS_ABSENT_VERIFIED' if task_absent else states['task'].state),actions_active=False,router=router,
            controllers=controllers,
            actuation_status=states['actuation'].data,motion_enabled=True,feedback_ready=True,
            control_snapshot=ctl,control_sha256=sha(ctx['old'].CONTROL))
    finally:
        window.sampling=False



def remote_binding(session,c,ctx,master,construction_path,construction,targets,out):
    import ast
    save = session.save
    name='/remote_grasp6d_node';params=construction['remote_parameters'];grasp=construction['master_grasp']
    require({k:master.getParam(k) for k in ctx['checker'].PARAMETERS}==params
        and master.getParam('/grasp')==grasp,'remote construction parameters changed')
    raw=(out/'remote_grasp6d_node.log').read_bytes()
    lines=[s for s in raw.decode(errors='replace').splitlines() if '[RemoteGrasp6D] authoritative /grasp loaded ' in s]
    require(len(lines)==1,'unique authoritative remote startup marker required')
    match=re.search(r'authoritative /grasp loaded key_count=(\d+) contact_height_registration_margin_enabled=(\S+) handeye_translation_max_error_m=(\S+)',lines[0])
    require(match and (int(match.group(1)),ast.literal_eval(match.group(2)),ast.literal_eval(match.group(3)))==
        (len(grasp),grasp.get('contact_height_registration_margin_enabled'),grasp.get('handeye_translation_max_error_m')),
        'remote loaded configuration marker differs')
    prefix=out/'remote_startup_prefix.log'
    fd=os.open(prefix,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'wb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    require(prefix.read_bytes()==raw,'remote startup log readback differs')
    identity=dict(targets[name],cmdline_sha256=hashlib.sha256((Path('/proc')/str(targets[name]['pid'])/'cmdline').read_bytes()).hexdigest())
    binding=dict(schema=1,node_name=name,master_uri='http://localhost:11311',parameters=params,
        allowed_backends=ctx['config']['allowed_backends'],client_contract_source=ctx['config']['client_contract_source'],
        node_identity=identity,construction_evidence_review_required=True,
        ready_health_not_implied_by_startup_binding=True,
        evidence=[dict(path=str(p),sha256=sha(p)) for p in (construction_path,prefix,out/'remote_grasp6d_node_source_start.json')],
        construction_provenance=dict(kind='same_original_entry_configuration',source_cache_pid_proof=True,memory_attestation=False))
    path=out/'remote_startup_binding.json';save(path,binding)
    return dict(path=str(path),sha256=sha(path))


def deploy(session, args):
    c,r,ctx = session.live_context(for_update=True); bundle = session.bundle()
    bundle['expected_processes'] = ctx['expected_processes']
    W, C = session.workspace, session.candidate
    PRODUCTION, ADDITIONS = session.production, session.tests
    EPOCH, SELECTION = session.expected_epoch, session.expected_selection
    STARTER = Path(session.data['starter']['path'])
    source_check, target_processes = r.source_check, r.target_processes
    graph, stop_one, observed_process = r.graph, r.stop_one, r.observed_process
    source_process_proof, fresh_new_health = r.source_process_proof, r.fresh_new_health
    save = session.save
    sample = lambda *values: sample_current(session,r,*values)
    c.fresh_remote_health = lambda ctx,out: original_remote_health(session,ctx,out)
    source_check(W,bundle['baseline_source_sha256'])
    out=args.output.resolve();c.storage(out);require(not out.exists(),'single-use transition exists');out.mkdir()
    bundle_path=out/'bundle.json';save(bundle_path,bundle)
    bundle_digest=sha(bundle_path)
    os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    serial=[0]
    def journal(stage,value):
        serial[0]+=1;save(out/('%03d_%s.json'%(serial[0],stage)),dict(recorded_ns=time.time_ns(),detail=value))
    result=dict(kind='three_compute_owner_reload',release_label=session.data['release_label'],passed=False,deployment_passed=False,
        installed_paths=[],shutdown_nodes=[],started_nodes=[],automatic_retry_allowed=False,
        bundle=str(bundle_path),bundle_sha256=bundle_digest,contract=str(session.path),contract_sha256=session.digest,
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,torque_disable_commands_sent=False)
    try:
        import rosgraph
        import rospy
        from std_srvs.srv import Trigger
        rospy.init_node('compute_owner_update',anonymous=True,disable_rosout=True);master=session.checked_master(c,ctx,rosgraph.Master(rospy.get_name()),rospy.get_name())
        targets={n:bundle['nodes'][n] for n in TARGETS};units=dict(ctx['prior_transition']['units'])
        before=graph(c,ctx,rospy,master,targets,units);aliases=before['auxiliary_nodes']
        initial=sample(c,ctx,rospy,master,targets);parameters=c.parameters(master,out,'before')
        result['remote_health_before']=c.fresh_remote_health(ctx,out/'remote_readiness_before')
        original={n:observed_process(ctx,n,bundle['expected_processes'][n]) for n in TARGETS}
        uris={n:master.lookupNode(n) for n in TARGETS}
        c.recheck(ctx,bundle['baseline_source_sha256']);graph(c,ctx,rospy,master,targets,units,aliases)
        sample(c,ctx,rospy,master,targets)
        for name in TARGETS:
            stop_one(c,ctx,rospy,master,name,targets[name],uris[name],bundle,journal)
            targets.pop(name);units.pop(name,None);result['shutdown_nodes'].append(name)
            graph(c,ctx,rospy,master,targets,units,aliases);sample(c,ctx,rospy,master,targets)
        require(not any(target_processes().values()),'all three old compute instances must exit before installation')
        c.same_parameters(parameters,c.parameters(master,out,'stopped'),out,'stopped')
        # Reuse the reviewed per-file atomic install/cache backup functions,
        # rebinding only their finite source set and cache enumerator.
        from types import SimpleNamespace
        installer=rebind(session.modules['installer'],c=SimpleNamespace(require=require,sha=sha,W=W,
            contract=SimpleNamespace(CHANGED=set(PRODUCTION)|set(ADDITIONS),C=C),target_caches=lambda:r.caches(c)))
        for relative in sorted(set(PRODUCTION)|set(ADDITIONS)):
            installer.atomic_install_file(relative,bundle['baseline_source_sha256'].get(relative),
                bundle['source_sha256'][relative],out/'source_backup',journal)
            result['installed_paths'].append(relative)
        result['cache_quarantine']=installer.quarantine_target_cache(out,journal)
        session.phase_sources=session.sources
        source_check(W,bundle['source_sha256']);sample(c,ctx,rospy,master,targets)
        graph(c,ctx,rospy,master,targets,units,aliases)
        config=ctx['config'];remote_params={k:master.getParam(k) for k in ctx['checker'].PARAMETERS}
        require(remote_params==config['remote_parameters'],'original remote startup config changed')
        construction=dict(schema=1,kind='three_compute_owner_construction',release_label=session.data['release_label'],output=str(out),
            bundle=str(bundle_path),bundle_sha256=bundle_digest,contract=str(session.path),contract_sha256=session.digest,processes={},
            remote_parameters=remote_params,master_grasp=master.getParam('/grasp'),
            epoch_ns=EPOCH,selection=SELECTION,source_sha256=bundle['source_sha256'])
        for name in TARGETS:
            process=deepcopy(original[name]);base=name.lstrip('/');logs=out/(base+'_ros');logs.mkdir()
            ros_args=[v for v in process['argv'] if ':=' in v and not v.startswith('__log:=')]
            ros_args.append('__log:='+str(logs/(base+'.log')))
            process.update(entry=ENTRIES[name],ros_args=ros_args,unit='alicia-compute-update-'+base+'-'+str(time.time_ns()),
                run_path=ctx['prior_construction']['processes'][name]['run_path'],
                original_sys_path0=ctx['prior_construction']['processes'][name]['original_sys_path0'])
            process['environment']['ROS_LOG_DIR']=str(logs);construction['processes'][name]=process
        path=out/'construction.json';save(path,construction)
        for name in TARGETS:
            process=construction['processes'][name];unit=process['unit'];base=name.lstrip('/')
            command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec','--property=Restart=no',
                '--property=WorkingDirectory='+process['cwd'],'--property=StandardOutput=append:'+str(out/(base+'.log')),
                '--property=StandardError=append:'+str(out/(base+'.log')),
                '/usr/bin/python3','-B',str(STARTER),'--construction',str(path),'--construction-sha256',sha(path),
                '--node',name,*process['ros_args']]
            c.recheck(ctx,bundle['source_sha256']);require(not target_processes()[name],'target reappeared before launch')
            journal('launch_intent',dict(node=name,unit=unit,command=command))
            subprocess.run(command,check=True,timeout=15.)
            end=time.monotonic()+40.
            while True:
                try:row=c.identity(ctx,master,rospy.get_name(),name);break
                except (rosgraph.MasterError,ConnectionRefusedError):
                    require(time.monotonic()<end,'new node did not register');time.sleep(.1)
            require(row!=bundle['nodes'][name],'new node has old identity')
            targets[name]=row;units[name]=unit;result['started_nodes'].append(name)
            graph(c,ctx,rospy,master,targets,units,aliases);sample(c,ctx,rospy,master,targets)
        acknowledgements={}
        for service,name in (('/grasp_mode/reset_remote','/remote_grasp6d_node'),('/grasp_mode/reset_task','/grasp_task_node')):
            rospy.wait_for_service(service,timeout=10.)
            require(dict(master.getSystemState()[2]).get(service)==[name]
                and c.identity(ctx,master,rospy.get_name(),name)==targets[name],'reset service owner changed')
            sample(c,ctx,rospy,master,targets);journal('reset_intent',dict(service=service,identity=targets[name],selection=SELECTION))
            reply=rospy.ServiceProxy(service,Trigger)();ack=json.loads(reply.message)
            require(reply.success and ack.get('ready') is True and c.selection_identity({k:ack[k] for k in SELECTION})==SELECTION,
                'reset did not acknowledge existing selection')
            acknowledgements[service]=ack;journal('reset_result',dict(service=service,ack=ack))
        binding=remote_binding(session,c,ctx,master,path,construction,targets,out)
        result.update(passed=True,construction=str(path),construction_sha256=sha(path),targets=targets,units=units,
            initial_idle=initial,final_idle=sample(c,ctx,rospy,master,targets),before=before,
            after=graph(c,ctx,rospy,master,targets,units,aliases),parameter_snapshot=parameters,
            reset_acknowledgements=acknowledgements,remote_binding=binding,
            source_process_proof=source_process_proof(c,ctx,bundle,construction,targets,out),
            remote_health=fresh_new_health(c,ctx,binding,out/'remote_readiness_after'),recorded_ns=time.time_ns())
        c.same_parameters(parameters,c.parameters(master,out,'after'),out,'after')
        c.recheck(ctx,bundle['source_sha256']);save(out/'result.json',result)
    except BaseException as exc:
        states={}
        for relative in sorted(set(PRODUCTION)|set(ADDITIONS)):
            path=W/relative
            actual=sha(path) if path.is_file() else None
            states[relative]=dict(actual_sha256=actual,
                state=('installed_candidate' if actual==bundle['source_sha256'][relative] else
                       'baseline' if actual==bundle['baseline_source_sha256'].get(relative) else 'unexpected'))
        result.update(passed=False,error=repr(exc),recorded_ns=time.time_ns(),runtime_acceptance_passed=False,
            manual_mode_must_remain=True,actual_file_states=states,actual_target_processes=target_processes())
        save(out/'failure.json',result);raise


def accept(session, args):
    session.phase_sources=session.sources
    c,r,ctx=session.live_context()
    source_check,graph=r.source_check,r.graph
    source_process_proof,fresh_new_health=r.source_process_proof,r.fresh_new_health
    sample=lambda *values:sample_current(session,r,*values)
    save=session.save
    bundle=session.bundle();source_check(session.workspace,bundle['source_sha256'])
    path=args.reload.resolve(strict=True);require(session.output_root in path.parents and sha(path)==args.reload_sha256,'reload receipt changed')
    prior=session.read_runtime(path);require(prior['passed'] and prior['kind']=='three_compute_owner_reload'
        and prior['contract_sha256']==session.digest and prior['contract']==str(session.path) and set(prior['targets'])==set(TARGETS),'reload incomplete')
    require(sha(prior['bundle'])==prior['bundle_sha256'], 'runtime bundle changed')
    actual_bundle=session.read_runtime(prior['bundle'])
    require(actual_bundle['contract']==str(session.path) and actual_bundle['contract_sha256']==session.digest
        and actual_bundle['source_sha256']==session.sources
        and actual_bundle['baseline_source_sha256']==session.baseline_sources
        and actual_bundle['preserved']==session.preserved, 'runtime bundle source/identity closure differs')
    out=args.output.resolve();c.storage(out);require(not out.exists(),'single-use acceptance output exists');out.mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    import rosgraph
    import rospy
    from sensor_msgs.msg import Image
    rospy.init_node('compute_owner_accept',anonymous=True,disable_rosout=True);master=session.checked_master(c,ctx,rosgraph.Master(rospy.get_name()),rospy.get_name())
    ctx['idle_reference']=dict(names=prior['initial_idle']['names'],positions=prior['initial_idle']['positions'])
    targets,units=prior['targets'],prior['units'];construction=session.read_runtime(prior['construction'])
    require(sha(prior['construction'])==prior['construction_sha256'],'construction changed')
    before=graph(c,ctx,rospy,master,targets,units,prior['after']['auxiliary_nodes'])
    idle=sample(c,ctx,rospy,master,targets)
    c.same_parameters(prior['parameter_snapshot'],c.parameters(master,out,'accepted'),out,'accepted')
    health=fresh_new_health(c,ctx,prior['remote_binding'],out/'remote_readiness')
    proof=source_process_proof(c,ctx,bundle,construction,targets,path.parent)
    camera={}
    for kind,topic in (('rgb','/supervisor/camera/color/image_raw'),('depth','/supervisor/camera/depth/image_raw')):
        msg=rospy.wait_for_message(topic,Image,timeout=5.);camera[kind]=ctx['v'].camera_sample(msg,rospy.Time.now().to_nsec())
    final=graph(c,ctx,rospy,master,targets,units,before['auxiliary_nodes']);c.recheck(ctx,bundle['source_sha256'])
    result=dict(kind='three_compute_owner_acceptance',generation=session.data['release_label'],deployment_passed=True,
        full_restart=False,physical_grasp_verified=False,automatic_attempt_authorized=False,
        fresh_scene_and_dependency_gate_required_before_automatic_attempt=True,
        preserved_count=14,replaced_count=3,preserved=bundle['preserved'],targets=targets,
        auxiliary_nodes=final['auxiliary_nodes'],
        full_runtime_nodes=final['node_identities'],before=before,after=final,
        reload=str(path),reload_sha256=args.reload_sha256,bundle=prior['bundle'],bundle_sha256=prior['bundle_sha256'],contract=str(session.path),contract_sha256=session.digest,
        source_sha256=bundle['source_sha256'],source_process_proof=proof,gateway_compatibility=bundle['gateway_compatibility'],
        remote_dependency=health,camera=camera,camera_frame_check_does_not_grant_scene_clearance=True,
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,torque_disable_commands_sent=False,**idle)
    save(out/'result.json',result)


def sample_current(session, runtime, base, ctx, rospy, master, targets):
    absent = '/grasp_task_node' not in targets
    if absent:
        require(not runtime.target_processes()['/grasp_task_node'] and not any('/grasp_task_node' in owners
            for rows in master.getSystemState() for _,owners in rows), 'task absence not proven')
    return sample_idle(ctx, rospy, master, session.expected_selection, session.expected_epoch, absent)


def original_remote_health(session, ctx, out):
    session.storage(Path(out)/'result.json'); Path(out).mkdir(exist_ok=False)
    result=ctx['checker'].evaluate(ctx['binding'],Path(out)); session.save(Path(out)/'result.json',result)
    require(result['passed'] and result['dependency_ready'],'current retained remote dependency not ready')
    return dict(path=str(Path(out)/'result.json'),sha256=sha(Path(out)/'result.json'),passed=True,
        health_not_prediction_or_geometry_proof=True)


def invocation_failure(args, session, error, output_existed):
    """Keep an existing deploy failure; never overwrite an earlier invocation."""
    out=args.output.resolve()
    if output_existed:
        return
    if session is not None:
        session.storage(out/'failure.json')
    else:
        # Contract construction may fail before its storage policy is usable.
        # Such failures still stay on the explicitly required external disk.
        mount=EXPERIMENT_ROOT.parent.parent
        require(mount.is_mount() and EXPERIMENT_ROOT in out.parents
            and out.parent.is_dir() and out.parent.stat().st_dev==mount.stat().st_dev
            and not os.statvfs(mount).f_flag & os.ST_RDONLY,
            'external failure receipt unavailable; no fallback')
    out.mkdir(exist_ok=True)
    path=out/'failure.json'
    if path.exists():
        return
    value=dict(kind='three_compute_owner_invocation_failure',passed=False,
        runtime_acceptance_passed=False,action=args.action,recorded_ns=time.time_ns(),
        contract=str(args.contract.resolve()),contract_sha256=args.contract_sha256,
        error=repr(error),traceback=traceback.format_exc(),automatic_retry_allowed=False,
        partial_state_evidence_must_be_inspected=True,
        completed_result_present=(out/'result.json').exists())
    if hasattr(error,'compute_owner_cleanup_error'):
        value['cleanup_error']=error.compute_owner_cleanup_error
    if session is not None:
        session.save(path,value)
    else:
        raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
        with path.open('xb') as stream:
            stream.write(raw);stream.flush();os.fsync(stream.fileno())
        require(path.read_bytes()==raw,'failure receipt readback differs')


def main(argv=None):
    require(not sys.flags.optimize, 'unoptimized Python required')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('plan','deploy','accept'))
    parser.add_argument('--contract',type=Path,required=True)
    parser.add_argument('--contract-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--reload',type=Path)
    parser.add_argument('--reload-sha256')
    args=parser.parse_args(argv)
    session=None;output_existed=args.output.resolve().exists()
    try:
        with UpdateTransaction(args.contract,args.contract_sha256) as session:
            if args.action=='plan':
                session.recheck()
                out=args.output.resolve();session.storage(out/'result.json');out.mkdir(exist_ok=False)
                session.save(out/'result.json',session.summary())
            elif args.action=='deploy':
                deploy(session,args)
            else:
                require(args.reload is not None and args.reload_sha256,'exact returned reload required')
                accept(session,args)
    except BaseException as error:
        try:
            invocation_failure(args,session,error,output_existed)
        except BaseException as receipt_error:
            print('Failed to save invocation failure: '+repr(receipt_error),file=sys.stderr)
        raise


if __name__=='__main__':
    main()

