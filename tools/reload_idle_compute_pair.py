#!/usr/bin/env python3
"""Replace reviewed perception/inference Python sources while the robot is idle.

Only the two compute nodes are stopped. The manifest names the current runtime,
source delta and validation evidence; it does not inherit a historical release.
No mode selection, motion, gripper or driver command is issued.
"""
import argparse
import ast
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time
import types
import xmlrpc.client

TARGETS = ('/unknown_tabletop_perception', '/remote_grasp6d_node')
ENTRIES = dict(zip(TARGETS, ('unknown_tabletop_perception.py', 'mode_aware_remote_grasp6d.py')))
ALLOWED = {'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/source_connected_owner.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'}
PRIMITIVES = {
    '.runtime/restore_gen62_after_handoff_20261022.py': 'f1d5ad558dfd40cf5f7e59820b6ab84b55b40933236014debe41229c155ab61d',
    '.runtime/verify_active_remote_readiness_20261020.py': 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'}


class RpcTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3.0
        return connection


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    raw = (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n').encode()
    Path(path).write_bytes(raw)
    require(Path(path).read_bytes() == raw, 'evidence write failed')


def read_ref(ref):
    require(sha(ref['path']) == ref['sha256'], 'input changed: ' + str(ref['path']))
    return json.loads(Path(ref['path']).read_text())


def module(path, name, expected):
    require(sha(path) == expected, 'reviewed primitive changed')
    result = types.ModuleType(name)
    result.__file__ = str(path)
    exec(compile(Path(path).read_bytes(), str(path), 'exec'), result.__dict__)
    return result


def process(pid):
    p = Path('/proc') / str(pid)
    stat = (p/'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, parent_pid=int(stat[1]), start_ticks=stat[19],
        cmdline_sha256=hashlib.sha256((p/'cmdline').read_bytes()).hexdigest())


def node(master, name):
    uri = master.lookupNode(name)
    code, _, pid = xmlrpc.client.ServerProxy(uri,transport=RpcTransport()).getPid('/idle_compute_reload')
    require(code == 1, 'node PID unavailable: ' + name)
    return dict(process(pid), uri=uri)


def child(path, digest):
    data = read_ref(dict(path=path, sha256=digest))
    require(sha(__file__)==data['launcher_sha256'],'launcher changed before start')
    for source, expected in data['sources'].items():
        require(sha(source) == expected, 'installed source changed before start')
    require(data['node'] in TARGETS and Path(data['argv'][1]).name == ENTRIES[data['node']],
            'unexpected compute entry')
    Path(path).unlink()  # The full environment is transient, never a public log.
    os.chdir(data['cwd'])
    save(data['receipt'], dict(node=data['node'], pid=os.getpid(), sources=data['sources'],
        recorded_ns=time.time_ns(), original_entry=data['argv'][1],
        source_checked_before_exec=True, memory_attestation=False))
    os.execve(data['argv'][0], data['argv'], data['environment'])


def main(args):
    require(not sys.flags.optimize, 'unoptimized Python required')
    plan = read_ref(dict(path=args.manifest, sha256=args.sha256))
    require(sha(__file__)==plan['launcher_sha256'],'launcher changed')
    w = Path(plan['workspace']).resolve(strict=True)
    out = Path(plan['output']).resolve()
    mount = Path('/media/zhuyupei/SanStand')
    require(mount.is_mount() and mount in out.parents and out.parent.stat().st_dev == mount.stat().st_dev,
            'SanStand is required')
    require(not out.exists(), 'output already exists')
    baseline = read_ref(plan['runtime'])
    binding = read_ref(plan['remote_binding'])
    require({row['relative'] for row in plan['changes']} <= ALLOWED and plan['changes'], 'source delta out of scope')
    require(len({row['relative'] for row in plan['changes']}) == len(plan['changes']), 'duplicate source change')
    for row in plan['changes']:
        require(sha(w/row['relative']) == row['before_sha256']
                and sha(row['candidate']) == row['after_sha256'], 'source delta changed')
    validated_sources = {}
    def collect_sources(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if isinstance(item, str) and len(item) == 64:
                    validated_sources[key] = item
                else:
                    collect_sources(item)
        elif isinstance(value, list):
            for item in value:
                collect_sources(item)
    for ref in plan['validation']:
        evidence = read_ref(ref)
        require(evidence['passed'] is True, 'validation did not pass')
        collect_sources(evidence.get('source_sha256', {}))
    for row in plan['changes']:
        require(any(path.endswith(row['relative']) and digest == row['after_sha256']
                    for path, digest in validated_sources.items()), 'changed source lacks frozen validation')
    out.mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'observer_ros')
    import rospy
    import rosgraph
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from std_srvs.srv import Trigger
    from alicia_grasp_modes.selection import parse_selection
    idle = module(w/'.runtime/restore_gen62_after_handoff_20261022.py', 'idle_contract_primitive',
        PRIMITIVES['.runtime/restore_gen62_after_handoff_20261022.py'])
    health = module(w/'.runtime/verify_active_remote_readiness_20261020.py', 'remote_health_primitive',
        PRIMITIVES['.runtime/verify_active_remote_readiness_20261020.py'])
    rospy.init_node('idle_compute_reload', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    lock, values, samples = threading.Lock(), {}, []
    topics = [('joint','/alicia_d/accepted_joint_states',JointState),
        ('epoch','/alicia_d/control_reference_epoch',Header),('manual','/gui/joint_direct_mode',Bool),
        ('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String),
        ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('actuation','/alicia_d/actuation_status',String),('ready','/alicia_d/feedback_ready',Bool),
        ('enabled','/alicia_d/motion_enabled',Bool)]
    def receive(key, msg):
        if msg._connection_header.get('callerid') != idle.OWNERS[key]:
            return
        received = rospy.Time.now().to_nsec()
        with lock:
            values[key] = (msg, received)
            if key == 'joint':
                samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(),received_ns=received,
                    names=list(msg.name),positions=list(msg.position),frame_id=msg.header.frame_id,
                    caller=msg._connection_header.get('callerid')))
                samples[:] = samples[-40:]
    subscriptions = [rospy.Subscriber(topic, kind, lambda msg,k=key:receive(k,msg),queue_size=40)
                     for key,topic,kind in topics]
    def snapshot():
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            with lock:
                ready = len(values)==len(topics) and len(samples)>=20
            if ready:
                break
            time.sleep(.02)
        for key,topic,kind in topics:
            if key not in ('joint','arm','hand'):
                receive(key,rospy.wait_for_message(topic,kind,timeout=3))
        with lock:
            state,history = dict(values),list(samples[-20:])
        evidence=idle.validate_idle_snapshot(history,state,rospy.Time.now().to_nsec(),baseline['epoch_ns'])
        require(state['manual'][0].data is True and rospy.get_param('/gui/joint_direct_mode') is True,
                'GUI manual mode and parameter required')
        require(parse_selection(rospy.get_param('/grasp_mode/selection')) == parse_selection(baseline['selection']),
                'selection changed')
        publishers=dict(master.getSystemState()[0])
        for key,topic,_ in topics:
            require(publishers.get(topic)==[idle.OWNERS[key]], 'feedback/control owner changed: '+topic)
        return dict(evidence=evidence,joints=history[-1])
    def retained():
        for name,expected in baseline['nodes'].items():
            if name not in TARGETS:
                require(node(master,name)==expected,'retained node changed: '+name)
    def boundary():
        retained()
        return snapshot()
    def target_graph():
        return {name:[sorted(topic for topic,owners in group if name in owners)
                      for group in master.getSystemState()] for name in TARGETS}
    report=dict(passed=False,started_ns=time.time_ns(),manifest=str(args.manifest),
        manifest_sha256=args.sha256,mode_commands_sent=False,motion_commands_sent=False,
        temperature_queried=False,driver_restarted=False,physical_grasp_verified=False)
    private=[]
    try:
        report['before']=boundary()
        old={name:node(master,name) for name in TARGETS}
        require(all(old[name]==baseline['nodes'][name] for name in TARGETS),'compute instance changed')
        original_graph=target_graph()
        system=master.getSystemState()
        original_owners=[{topic:sorted(owners) for topic,owners in group
                         if any(name in owners for name in TARGETS)} for group in system]
        config={key:master.getParam(key) for key in health.PARAMETERS}
        require(config==binding['parameters'],'remote configuration changed from original startup')
        grasp=master.getParam('/grasp')
        starts={}
        for name in TARGETS:
            p=Path('/proc')/str(old[name]['pid'])
            env=dict(v.decode().split('=',1) for v in (p/'environ').read_bytes().split(b'\0') if v)
            argv=[v.decode() for v in (p/'cmdline').read_bytes().split(b'\0') if v]
            require(Path(argv[1]).resolve()==w/'devel/lib/alicia_grasp_modes'/ENTRIES[name], 'original entry differs')
            logdir=out/name.lstrip('/');logdir.mkdir()
            env['ROS_LOG_DIR']=str(logdir/'ros')
            argv=[arg if not arg.startswith('__log:=') else '__log:='+str(logdir/'node.log') for arg in argv]
            starts[name]=dict(node=name,argv=argv,environment=env,cwd=str((p/'cwd').resolve()),
                receipt=str(logdir/'source_start.json'),launcher_sha256=plan['launcher_sha256'])
        if not args.apply:
            report.update(passed=True,check_only=True)
            return
        # The last snapshot is taken immediately before stopping either owner.
        report['final_before_stop']=snapshot();retained()
        for name in TARGETS:
            boundary()
            require(node(master,name)==old[name],'compute changed before stop')
            code,_,_=xmlrpc.client.ServerProxy(old[name]['uri'],transport=RpcTransport()).shutdown(
                rospy.get_name(),'reviewed idle compute reload')
            require(code==1,'compute shutdown rejected')
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and any((Path('/proc')/str(row['pid'])).exists() for row in old.values()):
            time.sleep(.05)
        require(not any((Path('/proc')/str(row['pid'])).exists() for row in old.values()),'old compute process did not exit')
        require(all(not owners for groups in target_graph().values() for owners in groups),
                'old compute ROS ownership remains')
        boundary()
        sources={}
        for row in plan['changes']:
            boundary()
            target=w/row['relative'];candidate=Path(row['candidate'])
            require(sha(target)==row['before_sha256'] and sha(candidate)==row['after_sha256'],'source changed before install')
            backup=out/'preimage'/row['relative'];backup.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(target,backup)
            temporary=target.with_name(target.name+'.reviewed-reload')
            require(not temporary.exists(),'source staging path exists')
            temporary.write_bytes(candidate.read_bytes());os.chmod(temporary,target.stat().st_mode)
            os.replace(temporary,target);require(sha(target)==row['after_sha256'],'installed source differs')
            for cache in (target.parent/'__pycache__').glob(target.stem+'.*.pyc'):
                saved=out/'cache_before'/row['relative']/cache.name;saved.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(cache,saved);require(sha(cache)==sha(saved),'cache backup failed');cache.unlink()
            sources[str(target)]=row['after_sha256']
        report['installed_sources']=sources
        report['units']={}
        for name,data in starts.items():
            boundary()
            require(not any(name in owners for group in master.getSystemState()
                            for _,owners in group),'compute name reappeared before start')
            data['sources']=sources
            p=out/(name.lstrip('/')+'.private-start.json')
            fd=os.open(p,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
            with os.fdopen(fd,'w') as stream:json.dump(data,stream)
            private.append(p)
            unit='alicia-compute-'+name.lstrip('/')+'-'+str(time.time_ns())
            command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec','--property=Restart=no',
                '--property=StandardOutput=append:'+str(out/name.lstrip('/')/'console.log'),
                '--property=StandardError=append:'+str(out/name.lstrip('/')/'console.log'),
                '/usr/bin/python3','-B',str(Path(__file__).resolve()),'--child',str(p),'--sha256',sha(p)]
            subprocess.run(command,check=True,timeout=10,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            report['units'][name]=unit
        startup_deadline=time.monotonic()+35
        startup_error=None
        while time.monotonic()<startup_deadline:
            try:
                new={name:node(master,name) for name in TARGETS}
                for name,row in new.items():
                    receipt=json.loads((out/name.lstrip('/')/'source_start.json').read_text())
                    require(receipt['pid']==row['pid'] and row['pid']!=old[name]['pid'],'unexpected new compute process')
                require(target_graph()==original_graph,'compute graph is still initializing')
                break
            except (OSError,ValueError,RuntimeError,rosgraph.MasterError) as exc:
                startup_error=repr(exc)
                time.sleep(.1)
        else:
            raise RuntimeError('compute startup incomplete: '+str(startup_error))
        service_owners=dict(master.getSystemState()[2])
        require(service_owners.get('/grasp_mode/reset_remote')==['/remote_grasp6d_node'],
                'remote reset service owner differs')
        boundary()
        reply=rospy.ServiceProxy('/grasp_mode/reset_remote',Trigger)()
        require(reply.success and parse_selection(json.loads(reply.message))==parse_selection(baseline['selection']),
                'new remote did not acknowledge the unchanged selection')
        report['reset_ack']=json.loads(reply.message)
        require(target_graph()==original_graph,'new compute topic/service graph differs')
        for expected,group in zip(original_owners,master.getSystemState()):
            current=dict(group)
            require(all(sorted(current.get(topic,[]))==owners for topic,owners in expected.items()),
                    'compute topic/service ownership differs')
        report['after']=boundary()
        require(report['after']['joints']['names']==report['before']['joints']['names'] and
            max(abs(a-b) for a,b in zip(report['after']['joints']['positions'],report['before']['joints']['positions']))<=.004,
            'joint position changed during reload')
        require(master.getParam('/grasp')==grasp and {k:master.getParam(k) for k in health.PARAMETERS}==config,
                'construction configuration changed')
        console=out/'remote_grasp6d_node'/'console.log'
        prefix=console.read_bytes()
        markers=[line for line in prefix.decode(errors='replace').splitlines()
                 if '[RemoteGrasp6D] authoritative /grasp loaded ' in line]
        require(len(markers)==1,'new remote authoritative configuration marker missing or repeated')
        marker=re.search(r'key_count=(\d+) contact_height_registration_margin_enabled=(\S+) handeye_translation_max_error_m=(\S+)',markers[0])
        require(marker is not None and int(marker.group(1))==len(grasp)
            and ast.literal_eval(marker.group(2))==grasp.get('contact_height_registration_margin_enabled')
            and ast.literal_eval(marker.group(3))==grasp.get('handeye_translation_max_error_m'),
            'new remote construction configuration marker differs')
        (out/'remote_startup_prefix.log').write_bytes(prefix)
        new_binding=dict(schema=1,node_name='/remote_grasp6d_node',master_uri=binding['master_uri'],
            allowed_backends=binding['allowed_backends'],client_contract_source=binding['client_contract_source'],
            construction_evidence_review_required=True,ready_health_not_implied_by_startup_binding=True,
            parameters=config,node_identity={k:v for k,v in new['/remote_grasp6d_node'].items() if k!='uri'},
            source_sha256=sources,selection=baseline['selection'],
            startup_prefix_sha256=sha(out/'remote_startup_prefix.log'),
            original_binding=plan['remote_binding'],memory_attestation=False,
            evidence=[dict(path=str(p),sha256=sha(p)) for p in (
                args.manifest,out/'remote_startup_prefix.log',out/'remote_grasp6d_node'/'source_start.json')],
            construction='preserved original entry/environment/parameters; fresh PID and startup marker')
        save(out/'remote_startup_binding.json',new_binding)
        health_binding=health.load_binding(out/'remote_startup_binding.json',sha(out/'remote_startup_binding.json'))
        report['remote_health']=health.evaluate(health_binding,out)
        require(report['remote_health']['passed'],'remote dependency not ready: '+str(report['remote_health'].get('error')))
        require(node(master,'/remote_grasp6d_node')==new['/remote_grasp6d_node']
                and {k:master.getParam(k) for k in health.PARAMETERS}==config,'remote changed during health probe')
        report.update(passed=True,nodes_before=old,nodes_after=new,remote_parameters=config,
            configuration_bound_to_new_start=True)
    except Exception as exc:
        report['error']=repr(exc)
        raise
    finally:
        for p in private:
            if p.exists():p.unlink()
        report['finished_ns']=time.time_ns()
        save(out/'result.json',report)
        for subscription in subscriptions:subscription.unregister()
        rospy.signal_shutdown('compute reload finished')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--sha256',required=True)
    parser.add_argument('--apply',action='store_true')
    parser.add_argument('--child',type=Path)
    arguments=parser.parse_args()
    if arguments.child:
        child(arguments.child,arguments.sha256)
    else:
        require(arguments.manifest is not None,'manifest required')
        main(arguments)
