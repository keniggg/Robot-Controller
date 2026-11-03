"""Read-only current reference after verified explicit manual movement.

Preserve the old resume context and all its historical references. Reuse the
entire original snapshot with only the expected current epoch and additional
immutable manual-operation evidence in an isolated namespace.
"""
import argparse
import ast
import hashlib
import importlib.util
import inspect
from pathlib import Path
import sys

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
G = M/'实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
F = G/'contact_feedback_fix_r1'
ORIGINAL = W/'.runtime/gen62_resume_runtime_20261022.py'
ORIGINAL_SHA = 'd9f1ac014f09b5743a7bddedaa8219b386161d41bd5c3eacebf30afcd0ee3127'
PROOF = F/'manual_reference_evidence_r1/result.json'
REVIEW = F/'reconfirmed_reference_review_r1/result.json'
OBSERVATION = F/'manual_reconfirmation_observer_r1'
OUTPUT = G/'user_resume_r1/readiness_r2'
EPOCH = 1792668378941525117


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_original():
    require(sha(ORIGINAL) == ORIGINAL_SHA, 'original resume changed')
    spec = importlib.util.spec_from_file_location('gen62_historical_resume_unchanged', ORIGINAL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def current_context(original, proof_sha, review_sha):
    # The old EPOCH remains inside this unmodified historical function.
    ctx = original.context()
    require(sha(PROOF) == proof_sha, 'manual evidence review changed')
    proof = original.read(PROOF)
    require(sha(REVIEW) == review_sha, 'readonly helper review changed')
    review = original.read(REVIEW)
    require(review['passed'] is True and review['helper_sha256'] == sha(__file__),
            'independent review must bind this helper')
    require(proof['passed'] is True and proof['before_epoch_ns'] == original.EPOCH
            and proof['after_epoch_ns'] == EPOCH
            and proof['gui_positive_enable_count'] == 1 and proof['negative_enable_count'] == 0
            and proof['gui_command_count'] == 17 and proof['actual_sdk_motion_observed'] is True
            and proof['confirmed_observed'] is True and proof['observer_process_closed'] is True
            and proof['driver_callerid'] == '/alicia_d_driver_node'
            and proof['gui_callerid'] == '/alicia_supervisor_gui',
            'actual manual movement evidence incomplete')
    pins = {Path(p): h for p, h in proof['input_sha256'].items()}
    allowed_extra = {
        W/'.runtime/observe_gen62_manual_reconfirmation_20261022.py',
        G/'runtime_reload_r1/acceptance/result.json',
        W/'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp',
        W/'src/real-arm/alicia_d_driver/include/alicia_d_driver/actuation_confirmation.hpp',
    }
    require(all((F in p.parents or p in allowed_extra)
                and sha(p) == h for p, h in pins.items()), 'manual evidence input changed or outside finite roots')
    require(all(OBSERVATION/name in pins for name in
                ('events.jsonl', 'result.json', 'user_reply_actual_motion.json')),
            'actual observation and user reply must be bound')
    require(pins[OBSERVATION/'user_reply_actual_motion.json'] ==
            '65c7b4ff9c905d00c5e6306d8b87a43c02170dc90ba93c66389138744d6bda7a',
            'actual operator reply differs')
    reply = original.read(OBSERVATION/'user_reply_actual_motion.json')
    require(reply['source'] == 'actual_user_message' and
            reply['text'] == '已观察到机械臂实际移动，并离开桌边', 'operator reply missing')
    pins.update({PROOF: proof_sha, REVIEW: review_sha, ORIGINAL: ORIGINAL_SHA,
                 Path(__file__).resolve(): sha(__file__)})
    require(all(p not in ctx['pins'] or ctx['pins'][p] == h for p, h in pins.items()),
            'current evidence conflicts with historical pins')
    ctx['pins'].update(pins)
    original.recheck(ctx, ctx['release']['source_sha256'])
    return ctx


def main():
    if sys.flags.optimize:
        raise RuntimeError('unoptimized helper required')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--helper-sha256', required=True)
    parser.add_argument('--manual-proof-sha256', required=True)
    parser.add_argument('--review-sha256', required=True)
    args = parser.parse_args()
    require(sha(__file__) == args.helper_sha256 and args.output.resolve() == OUTPUT,
            'exact helper and fresh reference output required')
    original = load_original()
    tree = ast.parse(inspect.getsource(original.snapshot))
    require(len(tree.body) == 1 and isinstance(tree.body[0], ast.FunctionDef)
            and tree.body[0].name == 'snapshot', 'original snapshot shape changed')
    namespace = dict(original.__dict__)
    namespace.update(EPOCH=EPOCH, __file__=str(Path(__file__).resolve()),
                     context=lambda: current_context(original, args.manual_proof_sha256, args.review_sha256))
    exec(compile(tree, str(ORIGINAL)+'#GEN62_verified_current_manual_epoch', 'exec'), namespace)
    return namespace['snapshot'](args.output)


if __name__ == '__main__':
    sys.exit(main())
