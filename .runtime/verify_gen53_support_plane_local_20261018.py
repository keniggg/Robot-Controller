"""Bounded isolated pure-module GEN53 tests; not a complete regression."""
from pathlib import Path
import hashlib, importlib.util, json, os, platform, signal, struct, sys, time
W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
O = V/'gen53_support_plane_local_focus_r1'
B = W/'.runtime/gen53_baseline_source'
C = W/'.runtime/gen53_registration_candidate'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
    O.mkdir()
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:2])
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', TMPDIR=str(O))
    sys.dont_write_bytecode=True
    native=os.uname()
    platform.uname=lambda:platform.uname_result(*(tuple(native)+(native.machine,)))
    platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
    denied=[]
    def guard(event,args):
        bad=event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.exec','os.posix_spawn','os.fork','os.forkpty')
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            p=os.fsdecode(args[0]); flags=args[2] if len(args)>2 and isinstance(args[2],int) else 0
            mode=args[1] if len(args)>1 and isinstance(args[1],str) else ''
            bad|=(p.startswith('/dev/') and p not in ('/dev/null','/dev/urandom','/dev/random')) or p.startswith('/sys/class/thermal') or '/hwmon/' in p
            if p!='/dev/null' and (flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(c in mode for c in 'wax+')):
                bad|=not (Path(p).resolve()==O or O in Path(p).resolve().parents)
        if bad: denied.append(event); raise RuntimeError('offline boundary '+event)
    sys.addaudithook(guard)
    signal.signal(signal.SIGALRM,lambda *a:(_ for _ in ()).throw(TimeoutError('45 second local test budget')))
    signal.alarm(45)
    sys.path.insert(0,str(B/'src/alicia_flexible_grasp_supervisor/src'))
    import alicia_flexible_grasp.vision
    p=C/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/box_surface_registration.py'
    name='alicia_flexible_grasp.vision.box_surface_registration'
    spec=importlib.util.spec_from_file_location(name,p)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    alicia_flexible_grasp.vision.box_surface_registration=module
    import pytest
    test=C/'src/alicia_flexible_grasp_supervisor/tests/test_gen53_support_plane_registration.py'
    t=time.monotonic()
    rc=pytest.main([str(test),'-q','-p','no:cacheprovider','--basetemp',str(O/'tmp'),'--junitxml',str(O/'pytest.xml')])
    result=dict(passed=rc==0 and not denied,exit_code=int(rc),elapsed_sec=time.monotonic()-t,
        source_sha256={str(p):sha(p),str(test):sha(test)},test_scope='16 new pure module cases only',
        boundary_attempts=denied,full_regression=False,ROS_initialized=False,hardware_contacted=False,
        physical_grasp_verified=False,helper_sha256=sha(__file__))
    (O/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    signal.alarm(0)
    print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),passed=result['passed'])))
    return int(rc)
if __name__=='__main__':raise SystemExit(main())
