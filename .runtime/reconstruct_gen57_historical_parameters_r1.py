"""Offline launch expansion against the immutable historical all-parameter hash.

Never imports the startup entry or calls roslaunch launch/start/upload methods.
Only XML/YAML/textfile expansion and pinned read-only rosversion are allowed.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import sys
import time
import traceback
import xml.etree.ElementTree as ET

sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1]
M=Path('/media/zhuyupei/SanStand')
ROOT=M/'实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3'
H=ROOT/'full_restart_r1/startup_configuration.json'
D=ROOT/'actual_attempt_r1_geometry/historical_parameter_reconstruction_r1/attempt_2'
EXPECTED='d0c1444d3032b968f3f06952f4a2f3e381dc4b373e406189a7300b52a0c29787'

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def artifact(p): return dict(path=str(p),sha256=sha(p),bytes=Path(p).stat().st_size)
def write(name,data,canonical=False):
    raw=(json.dumps(data,sort_keys=canonical,ensure_ascii=False,allow_nan=False,
        **({} if canonical else {'indent':2}))+('' if canonical else '\n')).encode()
    p=D/name
    with p.open('xb') as f:f.write(raw);f.flush();os.fsync(f.fileno())
    assert p.read_bytes()==raw
    return artifact(p)

def main():
    assert M.is_mount() and D.parent.stat().st_dev==M.stat().st_dev
    D.mkdir(exist_ok=True)
    assert not (D/'result.json').exists() and not (D/'preflight.json').exists()
    historical=json.loads(H.read_text())
    assert historical['all_resolved_parameters_sha256']==EXPECTED
    sources=[];commands=[]
    for name,want in historical['launch_source_sha256'].items():
        path=Path(name);got=sha(path)
        sources.append(dict(path=name,expected_sha256=want,actual_sha256=got,matched=want==got))
        if path.suffix in ('.launch','.xml'):
            for tag in ET.parse(path).iter():
                if tag.tag=='param' and tag.get('command'):
                    commands.append(dict(source=name,name=tag.get('name'),command=tag.get('command')))
    assert all(x['matched'] for x in sources)
    assert sorted(x['command'] for x in commands)==['rosversion -d','rosversion roslaunch']
    cmd=Path(shutil.which('rosversion')).resolve()
    assert cmd==Path('/usr/local/bin/rosversion')
    catkin_source=(W/'devel/.catkin').read_text().strip()
    assert catkin_source==str(W/'src')
    # Equivalent local package lookup, without executing a startup shell or
    # contacting the master. ROS_IP only avoids DNS for config.master.uri.
    os.environ.update(ROS_PACKAGE_PATH=str(W/'src')+':/opt/ros/noetic/share',
        CMAKE_PREFIX_PATH=str(W/'devel')+':/opt/ros/noetic',ROS_DISTRO='noetic',
        ROS_IP='127.0.0.1',ROS_MASTER_URI='http://localhost:11311',
        ROS_HOME=str(D/'ros_home'),ROS_LOG_DIR=str(D/'ros_log'),
        PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1')
    allowed={('rosversion','roslaunch'),('rosversion','-d'),('uname','-p')}
    executable_sha={str(cmd):sha(cmd),'/usr/bin/uname':sha('/usr/bin/uname')}
    events=[];read_paths=set();blocked=[]
    def audit(event,args):
        if event.startswith('socket.') and event not in ('socket.gethostname',):
            blocked.append(dict(event=event));raise RuntimeError('offline reconstruction blocks '+event)
        if event in ('os.system','os.posix_spawn','os.fork','os.forkpty'):
            blocked.append(dict(event=event));raise RuntimeError('offline reconstruction blocks '+event)
        if event=='subprocess.Popen':
            executable,argv,cwd,env=args
            key=tuple(argv) if isinstance(argv,(list,tuple)) else (argv,)
            if key not in allowed:blocked.append(dict(event=event,argv=list(key)));raise RuntimeError('unreviewed command')
            resolved=str(Path(shutil.which(argv[0])).resolve())
            if resolved not in executable_sha or sha(resolved)!=executable_sha[resolved]:raise RuntimeError('command identity changed')
            events.append(dict(event=event,argv=list(argv),executable=resolved,sha256=executable_sha[resolved]))
        if event=='open':
            file,mode,flags=args
            if isinstance(file,(str,bytes,os.PathLike)):
                path=Path(os.fsdecode(file)).absolute()
                writing=bool((flags or 0)&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
                if writing and not (path==Path('/dev/null') or path==D or D in path.parents):
                    blocked.append(dict(event=event,path=str(path)));raise RuntimeError('write outside evidence directory')
                if not writing and path.suffix in ('.launch','.xml','.yaml','.yml','.urdf','.srdf','.xacro') and path.name!='package.xml':
                    read_paths.add(str(path.resolve()))
    write('preflight.json',dict(historical_configuration=artifact(H),launch_sources=sources,
        reviewed_param_commands=commands,command_executables=executable_sha,
        environment={k:os.environ[k] for k in ('ROS_PACKAGE_PATH','CMAKE_PREFIX_PATH','ROS_DISTRO','ROS_IP','ROS_MASTER_URI','ROS_HOME','ROS_LOG_DIR')},
        historical_entry_not_executed=True,helper=artifact(__file__)))
    sys.addaudithook(audit)
    start=time.monotonic();result={}
    try:
        import roslaunch
        cfg=roslaunch.config.load_config_default([historical['launch']],11311,verbose=False,assign_machines=False)
        all_params={k:v.value for k,v in cfg.params.items()}
        payload=json.dumps(all_params,sort_keys=True,ensure_ascii=False,allow_nan=False).encode()
        actual=hashlib.sha256(payload).hexdigest()
        match=actual==EXPECTED
        result.update(completed=True,historical_all_parameters_sha256=EXPECTED,
            reconstructed_all_parameters_sha256=actual,all_parameters_exact_hash_match=match,
            parameter_count=len(all_params))
        if match:
            selected={k:v for k,v in all_params.items() if k=='/robot_description' or k=='/handeye' or k.startswith('/handeye/')}
            assert '/robot_description' in selected and any(k.startswith('/handeye/') or k=='/handeye' for k in selected)
            result['all_parameters']=write('all_resolved_parameters.json',all_params,canonical=True)
            result['context_parameters']=write('context_parameters.json',selected)
            urdf=D/'robot_description.urdf'
            with urdf.open('x') as f:f.write(selected['/robot_description']);f.flush();os.fsync(f.fileno())
            result['robot_description']=artifact(urdf)
            result['context_parameter_names']=sorted(selected)
        else:
            result['not_proven_reason']='Current offline expansion differs from historical all-parameter hash; historical individual values cannot be asserted from this reconstruction.'
            result['current_only_parameters']=write('unverified_current_resolved_parameters.json',all_params,canonical=True)
        result['loaded_launch_sources']=[artifact(p) for p in sorted(set(cfg.roslaunch_files))]
        result['source_reads']=[artifact(p) for p in sorted(read_paths) if Path(p).is_file()]
        result['historically_pinned_sources_unchanged_after']=all(sha(x['path'])==x['expected_sha256'] for x in sources)
        assert result['historically_pinned_sources_unchanged_after']
        result['roslaunch_implementation']=[artifact(p) for p in
            ('/opt/ros/noetic/lib/python3/dist-packages/roslaunch/config.py',
             '/opt/ros/noetic/lib/python3/dist-packages/roslaunch/loader.py',
             '/opt/ros/noetic/lib/python3/dist-packages/roslaunch/xmlloader.py')]
    except Exception as exc:
        result.update(completed=False,error=repr(exc),traceback=traceback.format_exc())
    result.update(schema=1,scope='Historical parameter reconstruction by exact complete canonical JSON hash match; no live ROS evidence',
        historical_configuration=artifact(H),helper=artifact(__file__),wall_seconds=time.monotonic()-start,
        subprocess_events=events,blocked_events=blocked,ros_node_initialized=False,
        parameters_uploaded=False,master_or_network_queried=False,launch_startup_executed=False,
        protection_scope='Python parent audit blocks network and unexpected subprocesses; reviewed rosversion child is a read-only package/env version query, not a kernel sandbox.')
    output=write('result.json',result)
    print(json.dumps(dict(result=output,completed=result['completed'],all_parameters_exact_hash_match=result.get('all_parameters_exact_hash_match'),
        context_parameters=result.get('context_parameters'),error=result.get('error')),ensure_ascii=False))

if __name__=='__main__':main()
