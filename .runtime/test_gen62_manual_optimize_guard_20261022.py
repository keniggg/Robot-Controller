"""Finite offline entry check: optimized Python must stop before argument parsing."""
import ast
import difflib
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1')
OUT = F/'post_helpers_manual_optimize_r1'
MANUAL = W/'.runtime/restore_gen62_after_handoff_20261022.py'
POST = W/'.runtime/read_gen62_post_attempt_20261022.py'
BEFORE_SHA = 'c2eafc41ed90ed4c1c1aa440fd27a5d0f421a041627c3d3ead2dfdb6a84c12e4'
POST_SHA = 'fb65dde02e80636774ec677a21cbc595c82b41d396a27fb9548f36743456f7c0'
REASON = 'original assertion gates require unoptimized Python'
denied = []


def guard(event, args):
    forbidden = event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.posix_spawn','os.fork','os.exec')
    if event == 'import' and str(args[0]).split('.')[0] in ('rospy','rosgraph','rosnode','rosmaster','actionlib'):
        forbidden = True
    if forbidden:
        denied.append(event)
        raise RuntimeError('offline boundary: '+event)


sys.addaudithook(guard)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ParserReached(Exception):
    pass


def entry_probe(source, compile_optimization, runtime_optimization):
    namespace = {'__name__':'offline_manual_optimized_entry', '__file__':str(MANUAL)}
    exec(compile(source,str(MANUAL),'exec',optimize=compile_optimization),namespace)
    parser_calls = []

    def parser_sentinel():
        parser_calls.append(True)
        raise ParserReached('offline parser sentinel; no paths, ROS or controls accessed')

    namespace['parse_args'] = parser_sentinel
    namespace['sys'] = SimpleNamespace(flags=SimpleNamespace(optimize=runtime_optimization))
    try:
        namespace['main']()
    except RuntimeError as exc:
        assert str(exc) == REASON and not parser_calls
        return dict(explicitly_rejected=True,reason=str(exc),parser_calls=0)
    except ParserReached:
        return dict(explicitly_rejected=False,parser_calls=len(parser_calls))
    raise AssertionError('entry unexpectedly returned')


def main():
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and F.stat().st_dev == mount.stat().st_dev
    OUT.mkdir(exist_ok=False)
    started = time.monotonic()
    result = dict(kind='GEN62_manual_explicit_unoptimized_entry_guard',passed=False,
        ROS_initialized=False,live_nodes_contacted=False,controls_written=False,
        actual_helper_main_run=False,temperature_queried=False,production_modified=False,
        future_release_or_acceptance_created=False,boundary_attempts=denied)
    try:
        source = MANUAL.read_text()
        new = """def main():
    if sys.flags.optimize:
        raise RuntimeError('original assertion gates require unoptimized Python')
    args = parse_args()
    assert args.expected_generation == 6
"""
        old = """def main():
    args = parse_args()
    assert not sys.flags.optimize and args.expected_generation == 6
"""
        assert source.count(new) == 1
        before = source.replace(new,old)
        assert hashlib.sha256(before.encode()).hexdigest() == BEFORE_SHA
        assert sha(POST) == POST_SHA
        tree = ast.parse(source)
        main_node = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        assert isinstance(main_node.body[0],ast.If) and isinstance(main_node.body[0].body[0],ast.Raise)
        assert isinstance(main_node.body[1],ast.Assign)
        # -OO removes all assert gates. The explicit if/raise must survive and
        # reject both runtime optimization flag values before even the parser.
        probes = []
        for compile_level,runtime_level in ((1,1),(2,2),(2,1)):
            row = entry_probe(source,compile_level,runtime_level)
            assert row['explicitly_rejected'] is True and row['parser_calls'] == 0
            probes.append(dict(compile_optimization=compile_level,runtime_optimization=runtime_level,**row))
        normal = entry_probe(source,0,0)
        assert normal == dict(explicitly_rejected=False,parser_calls=1)
        previous = entry_probe(before,2,2)
        assert previous == dict(explicitly_rejected=False,parser_calls=1)
        patch = ''.join(difflib.unified_diff(before.splitlines(True),source.splitlines(True),
            fromfile=str(MANUAL)+'@'+BEFORE_SHA,tofile=str(MANUAL)+'@'+sha(MANUAL)))
        patch_path = OUT/'manual_entry_only.patch'
        patch_path.write_text(patch)
        assert not denied
        result.update(passed=True,source=dict(path=str(MANUAL),sha256=sha(MANUAL)),
            baseline_sha256=BEFORE_SHA,unchanged_post=dict(path=str(POST),sha256=POST_SHA),
            offline_test=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)),
            limited_diff=dict(path=str(patch_path),sha256=sha(patch_path)),optimized_rejections=probes,
            unoptimized_parser_reached=normal,baseline_optimized_missing_guard=previous,
            scope='Only the explicit main optimization guard changed; all remaining source bytes reconstruct the sealed c2eaf baseline.',
            note='Compiled namespaces with controlled sys.flags and a parser sentinel only. No actual ROS/helper execution or future evidence.')
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
    result['elapsed_seconds'] = time.monotonic()-started
    raw = (json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with (OUT/'result.json').open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert (OUT/'result.json').read_bytes() == raw
    print(json.dumps(dict(path=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),passed=result['passed'],
        source=result.get('source'),error=result.get('error')),ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
