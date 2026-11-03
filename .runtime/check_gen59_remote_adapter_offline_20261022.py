"""Offline startup-adapter semantics only; no ROS init, service, socket or process launch.

Tests source loading/sys.modules/relative imports and the unchanged production
main AST. Optionally imports real candidate definitions without constructing the
node, under a fail-closed audit guard. Does not claim runtime acceptance.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import types
import traceback


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True)
    args=p.parse_args(); w=Path(__file__).resolve().parents[1]
    mount=Path('/media/zhuyupei/SanStand'); out=args.output.resolve(); candidate=args.candidate.resolve(strict=True)
    assert mount.is_mount() and (mount/'实验记录/Robot-Controller/ros_log') in out.parents and not out.exists()
    out.mkdir(); assert out.stat().st_dev==mount.stat().st_dev
    sys.dont_write_bytecode=True
    blocked=[]; allowed_readonly_commands=[]
    os.environ['PATH']='/usr/bin:/bin'
    def audit(event,args):
        if event=='subprocess.Popen' and args[0]=='uname' and list(args[1])==['uname','-p'] and os.environ['PATH']=='/usr/bin:/bin':
            allowed_readonly_commands.append(dict(command=['uname','-p'],binary='/usr/bin/uname',
                binary_sha256=hashlib.sha256(Path('/usr/bin/uname').read_bytes()).hexdigest()))
            return
        if event.startswith(('socket.connect','socket.bind','socket.getaddrinfo')) or event in ('subprocess.Popen','os.system','os.fork','os.posix_spawn'):
            blocked.append(dict(event=event,command=(args[1] if event=='subprocess.Popen' else None)))
            raise RuntimeError('offline boundary: '+event)
        if event=='open':
            name,mode,flags=args
            if isinstance(name,(str,bytes,os.PathLike)):
                path=Path(os.fsdecode(name)).resolve()
                writing=(isinstance(mode,str) and any(x in mode for x in 'wax+')) or (isinstance(flags,int) and flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
                if writing and out not in path.parents and path!=Path('/dev/null'):
                    blocked.append(dict(event=event,path=str(path))); raise RuntimeError('offline write outside output')
                if str(path).startswith(('/dev/tty','/dev/serial','/dev/video','/sys/class/thermal')):
                    blocked.append(dict(event=event,path=str(path))); raise RuntimeError('offline device access')
    sys.addaudithook(audit)
    sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
    helper=w/'.runtime/start_gen59_remote_20261022.py'
    spec=importlib.util.spec_from_file_location('gen59_offline_startup_adapter',helper)
    adapter=importlib.util.module_from_spec(spec); spec.loader.exec_module(adapter)
    result=dict(passed=False,helper_sha256=sha(helper),candidate=str(candidate),checks={},blocked_events=blocked,
        live_ros_contacted=False,network_contacted=False,device_contacted=False,temperature_queried=False,
        source_sha256={},runtime_acceptance_claimed=False,allowed_readonly_commands=allowed_readonly_commands)
    try:
        pkg=types.ModuleType('gen59_adapter_fixture'); pkg.__path__=[str(out)]
        sys.modules[pkg.__name__]=pkg
        first=out/'fixture_a.py'; first.write_text('TOKEN = object()\n')
        a=adapter.load_verified_module(pkg.__name__+'.a',first,sha(first))
        second=out/'fixture_b.py'; second.write_text('from .a import TOKEN\n')
        b=adapter.load_verified_module(pkg.__name__+'.b',second,sha(second))
        assert b.TOKEN is a.TOKEN and pkg.a is a and pkg.b is b and sys.modules[pkg.__name__+'.a'] is a
        result['checks']['same_module_and_relative_import_identity']=True
        for label,name,digest in [('duplicate_module',pkg.__name__+'.a',sha(first)),('tampered_hash',pkg.__name__+'.bad','0'*64)]:
            try:
                adapter.load_verified_module(name,first,digest)
            except RuntimeError:
                result['checks'][label+'_rejected']=True
            else:
                raise AssertionError(label+' accepted')
        broken=out/'broken.py'; broken.write_text('raise RuntimeError("original startup import failure")\n')
        try:
            adapter.load_verified_module(pkg.__name__+'.broken',broken,sha(broken))
        except RuntimeError as exc:
            assert str(exc)=='original startup import failure' and pkg.__name__+'.broken' not in sys.modules
            result['checks']['original_import_failure_propagates']=True
        else:
            raise AssertionError('startup failure suppressed')
        entry=candidate/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
        tree=ast.parse(entry.read_bytes())
        block=tree.body[-1]
        expected=ast.parse("if __name__ == '__main__':\n    rospy.init_node('remote_grasp6d_node')\n    ModeAwareRemoteGrasp6DNode().spin()\n").body[0]
        assert ast.dump(block,include_attributes=False)==ast.dump(expected,include_attributes=False)
        wrap=ast.parse(helper.read_bytes())
        main=next(n for n in wrap.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        expected_calls=ast.parse("entry.rospy.init_node('remote_grasp6d_node')\nentry.ModeAwareRemoteGrasp6DNode().spin()\n").body
        assert [ast.dump(n,include_attributes=False) for n in main.body[-2:]]==[ast.dump(n,include_attributes=False) for n in expected_calls]
        result['checks']['production_main_exact_two_call_semantics']=True
        # No constructor or rospy.init_node is called: only definitions import.
        os.environ['ROS_PACKAGE_PATH']=str(candidate/'src')+':/opt/ros/noetic/share'
        os.environ['ROS_HOME']=str(out/'ros_home')
        os.environ['ROS_LOG_DIR']=str(out/'ros_log')
        for path in reversed([candidate/'src/alicia_grasp_modes/src',candidate/'src/alicia_flexible_grasp_supervisor/src',
                              w/'devel/lib/python3/dist-packages',Path('/opt/ros/noetic/lib/python3/dist-packages')]):
            sys.path.insert(0,str(path))
        import alicia_grasp_modes
        declarations=[('alicia_grasp_modes.observe_return_visual','src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py'),
            ('alicia_grasp_modes.reachable_observation_views','src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'),
            ('_gen59_reviewed_remote_entry','src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py')]
        loaded={}
        for name,relative in declarations:
            path=candidate/relative; digest=sha(path)
            loaded[name]=adapter.load_verified_module(name,path,digest)
            result['source_sha256'][relative]=digest
            assert Path(loaded[name].__file__).resolve()==path
        visual=loaded[declarations[0][0]]; reachable=loaded[declarations[1][0]]; remote=loaded[declarations[2][0]]
        assert reachable.missing_side_view_gain is visual.missing_side_view_gain
        assert remote.directed_missing_side_rotations is visual.directed_missing_side_rotations
        assert not remote.rospy.core.is_initialized()
        result['checks']['three_real_candidate_modules_same_definition_identity']=True
        result['checks']['rospy_not_initialized']=True
        assert not blocked
        result['passed']=True
    except BaseException as exc:
        result['error']=repr(exc)
        result['traceback']=traceback.format_exc()
    finally:
        result['recorded_ns']=time.time_ns()
        path=out/'result.json'; path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(dict(passed=result['passed'],result=str(path),sha256=sha(path)),ensure_ascii=False))
    if not result['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
