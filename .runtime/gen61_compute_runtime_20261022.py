"""Bounded GEN61 compute-owner adapter. Import/prepare are offline.

The only live entrypoints are explicit ``deploy`` and ``accept`` actions.
They replace task, remote and unknown once, preserving the other fourteen
instances. No automatic mode, start, motion, temperature or disable API exists.
"""
import argparse
import ast
from copy import deepcopy
import hashlib
import importlib.util
import inspect
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import xmlrpc.client

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M/'实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z'
E = N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
C = W/'.runtime/gen61_seed_wait_candidate_r1'
BASE = N/'verification/gen60_release_manifest.json'
BASE_SHA = 'bad5d6c82cdb8ba956f84e725e7b584219a37a8f67e6da3148dfdd5b24538217'
ACCEPTED = E.parent/'runtime_reload_r1/acceptance/result.json'
ACCEPTED_SHA = '5ae84ec14b8ae5092ad88fc27d4367b257de9fc0fa80092220ad7db7ce3bc7e6'
DESIGN = E/'runtime_design_r1/result.json'
DESIGN_SHA = 'ca00fa6075c21056d4415d78bc48bbfabc635321c93fdf6ff4c8ae972ee7db42'
COMMON = W/'.runtime/gen60_gateway_reload_common_20261022.py'
COMMON_SHA = 'e0ae999e1fff774cf2c30ebb38c3fd8b7063e5f19339e492beb74854ef56e456'
INSTALLER = W/'.runtime/install_reload_gen60_gateway_20261022.py'
INSTALLER_SHA = '17596c3ac78d4371f23c2a352dc9960b9440c71be19f5b4949e2a33f8c15c8c5'
STARTER = W/'.runtime/start_gen61_compute_20261022.py'
START59 = W/'.runtime/start_gen59_remote_20261022.py'
START59_SHA = 'f004ce79ad43df6bdfd1278e231f1e839972c3ece7664379dbd98f2df76237b8'
P = 'src/alicia_grasp_modes/'
PRODUCTION = tuple(P+'src/alicia_grasp_modes/'+name+'.py' for name in (
    'measured_mask_seeds', 'source_connected_owner', 'station_compute_reacquisition',
    'station_compute_wait_producer', 'station_compute_wait_remote', 'station_compute_wait_task',
    'arrival_fragment_group'))+(P+'scripts/unknown_tabletop_perception.py',)
ADDITIONS = (P+'tests/test_gen61_seed_insufficiency.py', P+'tests/test_gen61_station_seed_wait.py',
             P+'tests/fixtures/gen60_raw_only_seed_insufficient.npz',
             P+'tests/fixtures/gen60_raw_only_seed_insufficient.json')
TARGETS = ('/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception')
SELECTION = dict(mode='unknown', strategy='observe_return', generation=4, stamp_ns=1792649183269455671)
EPOCH = 1792639534222874592
ENTRIES = dict(zip(TARGETS, (P+'scripts/mode_aware_grasp_task.py',
    P+'scripts/mode_aware_remote_grasp6d.py', P+'scripts/unknown_tabletop_perception.py')))


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and E in path.parents, 'writable SanStand GEN61 output required')
    parent = path.parent
    while not parent.exists():
        parent = parent.parent
    require(parent.stat().st_dev == M.stat().st_dev and not os.statvfs(M).f_flag & os.ST_RDONLY,
            'output mount unavailable; no system-disk fallback')


def save(path, value):
    storage(path)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(Path(path).read_bytes() == raw, 'receipt readback differs')


def load(path, digest, name):
    require(sha(path) == digest, 'helper changed: '+str(path))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def pins_check(pins):
    require(all(sha(p) == h for p, h in pins.items()), 'frozen input changed')


def source_check(root, sources):
    for relative, digest in sources.items():
        path = Path(root)/relative
        require(not Path(relative).is_absolute() and '..' not in Path(relative).parts
                and path.is_file() and not path.is_symlink() and sha(path) == digest,
                'source closure changed: '+relative)


def gateway_compatibility():
    relative = P+'src/alicia_grasp_modes/station_compute_reacquisition.py'
    before, after = [ast.parse((root/relative).read_text()) for root in (W, C)]
    defs = lambda tree: {n.name: ast.dump(n, include_attributes=False) for n in tree.body
                         if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    old, new = defs(before), defs(after)
    changed = sorted(k for k in old if old[k] != new.get(k))
    require(changed == ['validate_station_pending_notice'], 'unexpected shared existing definition change')
    old_top = [ast.dump(n, include_attributes=False) for n in before.body
               if not isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    new_top = [ast.dump(n, include_attributes=False) for n in after.body
               if not isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    require(all(n in new_top for n in old_top), 'shared imports/constants changed')
    upstream = [P+'src/alicia_grasp_modes/observe_return_measurement.py',
        'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_compensation_gateway.py']
    require(all(sha(W/p) == sha(C/p) for p in upstream), 'gateway upstream path changed')
    return dict(route='observation_compensation_gateway -> parse_measurement_request -> '
        'validate_station_requalification -> station_requalification_witness -> validate_contract',
        unchanged_existing_definitions=sorted(set(old)-set(changed)), changed_existing_definitions=changed,
        original_top_level_statements_preserved=True, upstream_sha256={p:sha(W/p) for p in upstream},
        gateway_is_an_indirect_consumer=True, gateway_memory_version_attested=False)


def prepare(args):
    pins = {str(BASE):BASE_SHA, str(ACCEPTED):ACCEPTED_SHA, str(DESIGN):DESIGN_SHA,
        str(COMMON):COMMON_SHA, str(INSTALLER):INSTALLER_SHA,
        str(START59):START59_SHA,
        str(W/'.runtime/gen60_gateway_release_contract_20261022.py'):
            sha(W/'.runtime/gen60_gateway_release_contract_20261022.py'),
        str(Path(__file__).resolve()):sha(__file__), str(STARTER):sha(STARTER)}
    pins_check(pins)
    baseline, accepted, design = read(BASE), read(ACCEPTED), read(DESIGN)
    require(accepted['deployment_passed'] and accepted['generation'] == 'GEN60'
        and accepted['source_manifest_sha256'] == BASE_SHA
        and accepted['epoch_ns'] == design['epoch_ns'] == EPOCH
        and design['selection'] == SELECTION, 'accepted GEN60/design binding differs')
    nodes = accepted['full_runtime_nodes']
    require(len(nodes) == 17 and design['preserve_count'] == 14
        and set(nodes)-set(TARGETS) == set(design['preserve_nodes'])
        and all(nodes[n] == design['current_process_evidence'][n]['identity'] for n in TARGETS)
        and all(nodes[n] == row for n,row in design['preserve_nodes'].items())
        and all(design['original_node_respawn'][n] is False for n in TARGETS), 'three/14 identity partition differs')
    source_check(W, baseline['source_sha256'])
    sources = {p:sha(C/p) for p in baseline['source_sha256']}
    require({p for p in sources if sources[p] != baseline['source_sha256'][p]} == set(PRODUCTION),
            'production change set differs from reviewed eight')
    sources.update({p:sha(C/p) for p in ADDITIONS})
    refs = {}
    if not args.design_only:
        for key in ('manifest', 'full', 'review'):
            path, digest = getattr(args, key), getattr(args, key+'_sha256')
            require(path is not None and digest is not None, 'final '+key+' and SHA required')
            path = path.resolve(strict=True); require(E in path.parents and sha(path) == digest, key+' changed')
            refs[key] = dict(path=str(path), sha256=digest); pins[str(path)] = digest
        manifest, full, review = [read(refs[k]['path']) for k in ('manifest','full','review')]
        require(manifest['full_scope_ready'] and manifest['source_sha256'] == sources
            and manifest['baseline_live_source_sha256'] == baseline['source_sha256']
            and Path(manifest['workspace_root']).resolve() == C, 'final manifest binding differs')
        source_check(C, manifest['copied_input_sha256'])
        require(full['mode'] == 'full' and full['passed'] and full['full_regression_passed']
            and full['scope_complete'] and full['source_unchanged'] and full['source_sha256'] == sources
            and full['candidate_manifest_sha256'] == refs['manifest']['sha256']
            and full['summary'] == dict(tests=manifest['total_case_count'],failures=0,errors=0,skipped=0)
            and full['test_scope_files'] == len(manifest['context']['scope']), 'complete exact frozen regression required')
        require(review['passed'] is True and review['production_source_sha256'] == {p:sources[p] for p in PRODUCTION},
                'independent review must bind final eight production bytes')
        for name, digest_key in (('pytest.xml','xml_sha256'), ('plan.json','plan_sha256')):
            path = Path(refs['full']['path']).parent/name
            require(sha(path) == full[digest_key], 'regression artifact changed'); pins[str(path)] = full[digest_key]
    resolution = ['devel/lib/alicia_grasp_modes/'+Path(ENTRIES[n]).name for n in TARGETS]
    resolution += ['devel/lib/python3/dist-packages/'+p+'/__init__.py'
                   for p in ('alicia_grasp_modes','alicia_flexible_grasp')]
    # Finite source-proven cached keys. Unsubscription changes only the dead
    # exact caller API, never parameter values or another process's entries.
    legacy = W/'.runtime/reload_gen34_remote_20261016.py'
    pins[str(legacy)] = 'c601f2f0d87d133f7a7744e25664cc13f4ee7bb6edc9c2a764b35f4a10da6065'
    pins_check(pins)
    cached = set()
    for node in ast.parse(legacy.read_text()).body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='CACHED_KEYS' for t in node.targets):
            cached.update(ast.literal_eval(node.value))
    require('/rosout_disable_topics_generation' in cached,'prior bounded cleanup keys changed')
    cached.add('/rosout_disable_topics_generation/')
    for relative in baseline['source_sha256']:
        if not relative.endswith('.py') or '/tests/' in relative or '/test/' in relative:
            continue
        tree=ast.parse((W/relative).read_bytes())
        aliases={t.id for node in ast.walk(tree) if isinstance(node,ast.Assign)
                 and isinstance(node.value,ast.Attribute) and node.value.attr=='get_param_cached'
                 for t in node.targets if isinstance(t,ast.Name)}
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and (isinstance(node.func,ast.Attribute) and node.func.attr=='get_param_cached'
                         or isinstance(node.func,ast.Name) and node.func.id in aliases) and node.args
                    and isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str)
                    and node.args[0].value.startswith('/')):
                cached.add(node.args[0].value)
    # Unknown's reviewed source-context loop uses these exact finite suffixes.
    defaults=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_view_limits.py'
    for node in ast.parse(defaults.read_text()).body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='DEFAULT_OBSERVATION_VIEW_PARAMETERS' for t in node.targets):
            require(isinstance(node.value,ast.Dict),'view defaults mapping changed')
            keys=[ast.literal_eval(key) for key in node.value.keys]
            require(keys and all(type(key) is str for key in keys),'view default keys changed')
            cached.update('/grasp_6d/remote/'+key for key in keys)
    bundle = dict(schema=1, kind='GEN61_three_compute_bundle', deployment_ready=not args.design_only,
        created_ns=time.time_ns(), pins=pins, frozen_evidence=refs,
        baseline_source_sha256=baseline['source_sha256'], source_sha256=sources,
        production=list(PRODUCTION), additions=list(ADDITIONS), nodes=nodes,
        preserved=design['preserve_nodes'], expected_processes=design['current_process_evidence'],
        epoch_ns=EPOCH, selection=SELECTION, entries=ENTRIES,
        load_resolution_sha256={p:sha(W/p) for p in resolution},
        gateway_compatibility=gateway_compatibility(), cached_parameter_cleanup_keys=sorted(cached),
        runtime_acceptance_performed=False, hardware_acceptance_claimed=False)
    out=args.output.resolve();storage(out);require(not out.exists(), 'single-use output exists')
    out.mkdir();save(out/'bundle.json',bundle)
    print(json.dumps(dict(path=str(out/'bundle.json'),sha256=sha(out/'bundle.json'),
        deployment_ready=bundle['deployment_ready'], live_ros_contacted=False)))


def context(args):
    path=args.bundle.resolve(strict=True);require(E in path.parents and sha(path)==args.bundle_sha256,'bundle changed')
    bundle=read(path);require(bundle['kind']=='GEN61_three_compute_bundle' and bundle['deployment_ready'] is True,
        'only final tested bundle can enter live transition')
    pins_check(bundle['pins']);source_check(C,bundle['source_sha256']);source_check(W,bundle['load_resolution_sha256'])
    c=load(COMMON,COMMON_SHA,'gen61_preserved_gen60_common');ctx=c.context()
    accepted=read(ACCEPTED)
    require(ctx['previous']['control_snapshot']==accepted['control_snapshot'],'control ownership changed')
    require(all(ctx['config']['nodes'][n]['respawn'] is False
        and ctx['config']['nodes'][n]['required'] is False for n in TARGETS),
        'original launch must not respawn or terminate the retained core on compute exit')
    ctx.update(previous=accepted,nodes=bundle['nodes'],preserved=bundle['preserved'],
        expected_gateway=bundle['nodes']['/motion_gateway'])
    ctx['pins'].update(bundle['pins']);ctx['pins'][str(path)]=args.bundle_sha256
    c.recheck(ctx)
    return c,ctx,bundle


def idle_function(c, task_absent=False):
    """Reuse all GEN60 feedback checks; only generation and proven absent task differ."""
    tree=ast.parse(inspect.getsource(c.sample_idle));counts=dict(generation=0,topic=0,check=0,result=0)
    class Adapt(ast.NodeTransformer):
        def visit_Compare(self,node):
            if ast.dump(node,include_attributes=False)==ast.dump(ast.parse("selection['generation']==3",mode='eval').body,include_attributes=False):
                counts['generation']+=1;node.comparators[0]=ast.Constant(4)
            return self.generic_visit(node)
        def visit_List(self,node):
            if task_absent and len(node.elts)==10 and all(isinstance(v,ast.Tuple) for v in node.elts):
                if isinstance(node.elts[2].elts[0],ast.Constant) and node.elts[2].elts[0].value=='task':
                    node.elts.pop(2);counts['topic']+=1
            return self.generic_visit(node)
        def visit_UnaryOp(self,node):
            if task_absent and ast.dump(node,include_attributes=False)==ast.dump(ast.parse("not states['task'].active",mode='eval').body,include_attributes=False):
                counts['check']+=1;return ast.Constant(True)
            return self.generic_visit(node)
        def visit_Attribute(self,node):
            if task_absent and ast.dump(node,include_attributes=False)==ast.dump(ast.parse("states['task'].state",mode='eval').body,include_attributes=False):
                counts['result']+=1;return ast.Constant('PROCESS_ABSENT_VERIFIED')
            return self.generic_visit(node)
    tree=Adapt().visit(tree);ast.fix_missing_locations(tree)
    require(counts==dict(generation=1,topic=int(task_absent),check=int(task_absent),result=int(task_absent)),
        'idle primitive shape changed')
    namespace=dict(c.__dict__,SELECTION=SELECTION)
    exec(compile(tree,str(COMMON)+'#GEN61_selection4_task_presence','exec'),namespace)
    return namespace['sample_idle'],counts


def target_processes():
    found={n:[] for n in TARGETS}
    for folder in Path('/proc').iterdir():
        if not folder.name.isdigit():continue
        try:argv=(folder/'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError,ProcessLookupError):continue
        except PermissionError:
            require(folder.stat().st_uid!=os.getuid(),'unreadable same-user process');continue
        for name in TARGETS:
            if any(('__name:='+v).encode() in argv for v in (name,name.lstrip('/'))):
                found[name].append(int(folder.name))
    return found


def graph(c,ctx,rospy,master,targets,units,auxiliaries=None,graph_wait=3.):
    require(type(graph_wait) in (int,float) and math.isfinite(graph_wait) and 0.<=graph_wait<=3.,
        'graph poll wait must be finite and within original three-second limit')
    dep=ctx['previous'];v=ctx['v'];core=v.properties(dep['unit'])
    require(core==dep['core_service'] and v.process(master.getPid())==dep['master_process'],'core/master changed')
    sim=v.properties(dep['simulation_unit']);require(sim==dep['simulation_service'],'simulation changed')
    expected=dict(ctx['preserved'],**targets)
    end=time.monotonic()+graph_wait
    while True:
        system=master.getSystemState();names={n for rows in system for _,owners in rows for n in owners}-{rospy.get_name()}
        extra=names-set(expected)
        if set(expected)<=names and len(extra)<=1 and all(re.fullmatch('/move_group_commander_wrappers_[0-9]+',n) for n in extra):break
        require(time.monotonic()<end,'graph owners changed: '+repr(sorted(names^set(expected))));time.sleep(.05)
    rows={n:c.identity(ctx,master,rospy.get_name(),n) for n in sorted(expected)}
    require(rows==expected,'registered node identity changed')
    aliases={n:c.identity(ctx,master,rospy.get_name(),n) for n in extra}
    require(all(r==ctx['preserved']['/motion_gateway'] for r in aliases.values())
        and (auxiliaries is None or aliases==auxiliaries),'gateway alias changed')
    in_core={n:r for n,r in expected.items() if n not in units}
    config=dict(ctx['config'],persistent_nodes=sorted(in_core))
    configured=v.configured_nodes(in_core,config,core)
    services={}
    for name,unit in units.items():
        require(name in targets,'unexpected unit target')
        props=v.properties(unit);row=targets[name]
        require(int(props['MainPID'])==row['pid'] and v.in_group(row['pid'],props['ControlGroup']), 'new compute unit ownership changed')
        services[name]=props
    require(target_processes()=={n:([targets[n]['pid']] if n in targets else []) for n in TARGETS},
        'duplicate or unregistered target process')
    publishers=dict(system[0]);ownership={}
    for topic,owner in dict(ctx['old'].PUBLISHERS,**{
        '/alicia_controller/follow_joint_trajectory/status':'/bessica_d_hw_interface',
        '/hand_controller/follow_joint_trajectory/status':'/bessica_d_hw_interface'}).items():
        actual=publishers.get(topic,[])
        require((not actual if owner not in expected else
            owner in actual if topic=='/gui/joint_direct_mode' else actual==[owner]),'publisher owner changed: '+topic)
        ownership[topic]=actual
    c.service_owners(ctx,master)
    return dict(node_identities=rows,auxiliary_nodes=aliases,core_service=core,simulation_service=sim,
        services=services,publishers=ownership,configured_nodes=configured,
        gui_windows=v.gui_windows(expected['/alicia_supervisor_gui']['pid']))


def sample(c,ctx,rospy,master,targets):
    absent='/grasp_task_node' not in targets
    if absent:
        require(not target_processes()['/grasp_task_node'] and not any('/grasp_task_node' in owners
            for rows in master.getSystemState() for _,owners in rows),'task absence not proven')
    function,_=idle_function(c,absent)
    return function(ctx,rospy,master,SELECTION,EPOCH)


def caches(c):
    rows=[]
    for relative in PRODUCTION:
        source=W/relative;folder=source.parent/'__pycache__'
        paths=list(folder.glob(source.stem+'.*.pyc')) if folder.exists() else []
        if source.with_suffix('.pyc').exists():paths.append(source.with_suffix('.pyc'))
        rows.extend(c.cache_status(p,source.read_bytes(),source.stat(),importlib.util.MAGIC_NUMBER) for p in sorted(paths))
    return rows


def observed_process(ctx,name,expected):
    require(ctx['v'].process(expected['identity']['pid'])==expected['identity'],'old process identity changed')
    folder=Path('/proc')/str(expected['identity']['pid'])
    argv=(folder/'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')
    env=dict(v.split('=',1) for v in (folder/'environ').read_bytes().rstrip(b'\0').decode().split('\0'))
    require(argv==expected['argv'] and str((folder/'cwd').resolve())==expected['cwd'],'old command/cwd changed')
    require(all(env.get(k)==v for k,v in expected['environment'].items()),'old numerical/startup environment changed')
    return dict(identity=expected['identity'],argv=argv,cwd=expected['cwd'],environment=env)


def stop_one(c,ctx,rospy,master,name,row,uri,bundle,journal):
    require(c.identity(ctx,master,rospy.get_name(),name)==row and master.lookupNode(name)==uri,'shutdown identity changed')
    journal('shutdown_intent',dict(node=name,identity=row,uri=uri))
    try:
        reply=xmlrpc.client.ServerProxy(uri,transport=ctx['old'].Transport()).shutdown(rospy.get_name(),'reviewed GEN61 compute-only reload')
        journal('shutdown_result',dict(node=name,reply=reply))
    except Exception as exc:journal('shutdown_result',dict(node=name,error=repr(exc)))
    end=time.monotonic()+25.
    while time.monotonic()<end:
        try:alive=ctx['v'].process(row['pid'])==row
        except (FileNotFoundError,ProcessLookupError,RuntimeError):alive=False
        if not alive:break
        c.recheck(ctx);time.sleep(.1)
    require(not alive and not target_processes()[name],'exact old target did not exit')
    def stale():
        require(not target_processes()[name] and not any(name in owners for rows in master.getSystemState() for _,owners in rows),
            'dead target still owns topic/service')
        try:current=master.lookupNode(name)
        except Exception as exc:
            import rosgraph
            if isinstance(exc,rosgraph.MasterError):return False
            raise
        require(current==uri,'stale name URI changed')
        for _ in range(2):
            try:xmlrpc.client.ServerProxy(uri,transport=ctx['old'].Transport()).getPid(rospy.get_name())
            except ConnectionRefusedError:pass
            else:raise RuntimeError('old caller URI answered; cleanup prohibited')
        return True
    import rosgraph
    caller=rosgraph.Master(name)
    for key in bundle['cached_parameter_cleanup_keys']:
        if not stale():break
        c.recheck(ctx);require(master.getParam('/gui/joint_direct_mode') is True,'manual ownership changed')
        journal('unsubscribe_intent',dict(node=name,caller_api=uri,key=key))
        reply=caller.unsubscribeParam(uri,key)
        journal('unsubscribe_result',dict(node=name,key=key,reply=reply))
    require(not stale(),'unreviewed stale registration remains; no expanded cleanup')


def remote_binding(c,ctx,master,construction_path,construction,targets,out):
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
        construction_provenance=dict(kind='GEN61_same_original_entry_configuration',source_cache_pid_proof=True,memory_attestation=False))
    path=out/'remote_startup_binding.json';save(path,binding)
    return dict(path=str(path),sha256=sha(path))


def fresh_new_health(c,ctx,reference,out):
    binding=ctx['checker'].load_binding(Path(reference['path']),reference['sha256'])
    storage(out);out.mkdir();result=ctx['checker'].evaluate(binding,out);save(out/'result.json',result)
    require(result['passed'] and result['dependency_ready'],'current new remote dependency not ready')
    return dict(path=str(out/'result.json'),sha256=sha(out/'result.json'),passed=True,
        health_not_prediction_or_geometry_proof=True)


def source_process_proof(c,ctx,bundle,construction,targets,out):
    source_check(W,bundle['source_sha256']);source_check(W,bundle['load_resolution_sha256'])
    require(not any(row['usable'] for row in caches(c)),'usable changed-target bytecode reappeared')
    result={}
    for name,row in targets.items():
        receipt_path=out/(name.lstrip('/')+'_source_start.json');receipt=read(receipt_path)
        folder=Path('/proc')/str(row['pid']);command=(folder/'cmdline').read_bytes()
        require(receipt['node_identity']==dict(row,cmdline_sha256=hashlib.sha256(command).hexdigest())
            and receipt['source_sha256']==bundle['source_sha256'] and receipt['node']==name
            and receipt['construction_sha256']==sha(out/'construction.json'), 'source/start/PID binding changed')
        if name=='/remote_grasp6d_node':
            expected={P+'src/alicia_grasp_modes/'+module+'.py':bundle['source_sha256'][
                P+'src/alicia_grasp_modes/'+module+'.py'] for module in ('observe_return_visual','reachable_observation_views')}
            require(receipt['exact_source_preloads']==expected,'retained GEN59 source preload guarantee changed')
        env=dict(v.split('=',1) for v in (folder/'environ').read_bytes().rstrip(b'\0').decode().split('\0'))
        require(env==construction['processes'][name]['environment'],'new exact process environment differs')
        maps=(folder/'maps').read_text()
        require('/usr/lib/python3/dist-packages/numpy/core/_multiarray_umath' in maps
            and '/usr/lib/python3/dist-packages/cv2.' in maps
            and not any('/.local/' in line and ('numpy' in line or '/cv2' in line) for line in maps.splitlines()),
            'actual numerical library mapping differs')
        if name=='/unknown_tabletop_perception':
            require(str(W/'.runtime/realsense-2.56.5/pyrealsense2.') in maps,'unknown local SDK mapping differs')
        result[name]=dict(receipt=str(receipt_path),sha256=sha(receipt_path),identity=row,
            source_cache_process_proof=True,memory_attestation=False)
    return result


def deploy(args):
    c,ctx,bundle=context(args);source_check(W,bundle['baseline_source_sha256'])
    out=args.output.resolve();c.storage(out);require(not out.exists(),'single-use transition exists');out.mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    serial=[0]
    def journal(stage,value):
        serial[0]+=1;save(out/('%03d_%s.json'%(serial[0],stage)),dict(recorded_ns=time.time_ns(),detail=value))
    result=dict(kind='GEN61_three_compute_reload',passed=False,deployment_passed=False,
        installed_paths=[],shutdown_nodes=[],started_nodes=[],automatic_retry_allowed=False,
        bundle=str(args.bundle.resolve()),bundle_sha256=args.bundle_sha256,
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,torque_disable_commands_sent=False)
    try:
        import rosgraph
        import rospy
        from std_srvs.srv import Trigger
        rospy.init_node('gen61_compute_reload',anonymous=True,disable_rosout=True);master=rosgraph.Master(rospy.get_name())
        targets={n:bundle['nodes'][n] for n in TARGETS};units={'/remote_grasp6d_node':ctx['previous']['remote_unit']}
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
        installer=load(INSTALLER,INSTALLER_SHA,'gen61_reviewed_atomic_installer')
        from types import SimpleNamespace
        installer.c=SimpleNamespace(require=require,sha=sha,W=W,
            contract=SimpleNamespace(CHANGED=set(PRODUCTION)|set(ADDITIONS),C=C),target_caches=lambda:caches(c))
        for relative in sorted(set(PRODUCTION)|set(ADDITIONS)):
            installer.atomic_install_file(relative,bundle['baseline_source_sha256'].get(relative),
                bundle['source_sha256'][relative],out/'source_backup',journal)
            result['installed_paths'].append(relative)
        result['cache_quarantine']=installer.quarantine_target_cache(out,journal)
        source_check(W,bundle['source_sha256']);sample(c,ctx,rospy,master,targets)
        graph(c,ctx,rospy,master,targets,units,aliases)
        config=ctx['config'];remote_params={k:master.getParam(k) for k in ctx['checker'].PARAMETERS}
        require(remote_params==config['remote_parameters'],'original remote startup config changed')
        construction=dict(schema=1,kind='GEN61_three_compute_construction',output=str(out),
            bundle=str(args.bundle.resolve()),bundle_sha256=args.bundle_sha256,processes={},
            remote_parameters=remote_params,master_grasp=master.getParam('/grasp'),
            epoch_ns=EPOCH,selection=SELECTION,source_sha256=bundle['source_sha256'])
        for name in TARGETS:
            process=deepcopy(original[name]);base=name.lstrip('/');logs=out/(base+'_ros');logs.mkdir()
            ros_args=[v for v in process['argv'] if ':=' in v and not v.startswith('__log:=')]
            ros_args.append('__log:='+str(logs/(base+'.log')))
            process.update(entry=ENTRIES[name],ros_args=ros_args,unit='alicia-gen61-'+base+'-'+str(time.time_ns()),
                run_path=(str(W/'devel/lib/alicia_grasp_modes'/Path(ENTRIES[name]).name) if name!='/remote_grasp6d_node' else str(W/ENTRIES[name])),
                original_sys_path0=str(Path(process['argv'][1]).parent if name!='/remote_grasp6d_node' else W/'.runtime'))
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
        binding=remote_binding(c,ctx,master,path,construction,targets,out)
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


def accept(args):
    c,ctx,bundle=context(args);source_check(W,bundle['source_sha256'])
    path=args.reload.resolve(strict=True);require(E in path.parents and sha(path)==args.reload_sha256,'reload receipt changed')
    prior=read(path);require(prior['passed'] and prior['kind']=='GEN61_three_compute_reload'
        and prior['bundle_sha256']==args.bundle_sha256 and set(prior['targets'])==set(TARGETS),'reload incomplete')
    out=args.output.resolve();c.storage(out);require(not out.exists(),'single-use acceptance output exists');out.mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    import rosgraph
    import rospy
    from sensor_msgs.msg import Image
    rospy.init_node('gen61_compute_accept',anonymous=True,disable_rosout=True);master=rosgraph.Master(rospy.get_name())
    ctx['idle_reference']=dict(names=prior['initial_idle']['names'],positions=prior['initial_idle']['positions'])
    targets,units=prior['targets'],prior['units'];construction=read(prior['construction'])
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
    result=dict(kind='GEN61_three_compute_acceptance',generation='GEN61',deployment_passed=True,
        full_restart=False,physical_grasp_verified=False,automatic_attempt_authorized=False,
        fresh_scene_and_dependency_gate_required_before_automatic_attempt=True,
        preserved_count=14,replaced_count=3,preserved=bundle['preserved'],targets=targets,
        auxiliary_nodes=final['auxiliary_nodes'],
        full_runtime_nodes=final['node_identities'],before=before,after=final,
        reload=str(path),reload_sha256=args.reload_sha256,bundle=str(args.bundle.resolve()),bundle_sha256=args.bundle_sha256,
        source_sha256=bundle['source_sha256'],source_process_proof=proof,gateway_compatibility=bundle['gateway_compatibility'],
        remote_dependency=health,camera=camera,camera_frame_check_does_not_grant_scene_clearance=True,
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,torque_disable_commands_sent=False,**idle)
    save(out/'result.json',result)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=('prepare','deploy','accept'))
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--design-only',action='store_true')
    for key in ('manifest','full','review','bundle','reload'):
        parser.add_argument('--'+key,type=Path);parser.add_argument('--'+key+'-sha256')
    args=parser.parse_args()
    if args.action!='prepare':
        require(args.bundle is not None and args.bundle_sha256 and not args.design_only,'final immutable bundle required')
    if args.action=='accept':require(args.reload is not None and args.reload_sha256,'exact reload receipt required')
    globals()[args.action](args)


if __name__=='__main__':
    main()
