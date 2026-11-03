"""Offline post-attempt adapter binding and preserved-sampler contracts."""
import argparse
import ast
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import time
import unittest
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'.runtime'));sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
import read_gen60_post_attempt_20261022 as post
from test_gen60_gateway_reload_contracts_20261022 import OfflineGuard
RUNTIME_MAP=post.E60/'runtime_contracts_r7/result.json'


def fixture():
    prior=dict(mode='unknown',strategy='observe_return',generation=3,stamp_ns=1792644272392787456)
    selected=dict(prior,generation=4,stamp_ns=prior['stamp_ns']+1)
    entry=dict(kind='GEN60_remote_one_recorded_attempt_recorded',original_runner_returned=True,
        gate_returned_to_original_mode_line=True,automatic_retry=False,new_motion_authority_created=False,runner_returncode=2)
    exited=dict(runner_returncode=2,selection=selected,physical_success='unverified')
    authorization=dict(selection=deepcopy(selected),previous_selection=prior,selection_response=dict(success=True),
        selection_commit=dict(latched_selection_matched=True))
    return entry,exited,authorization,dict(selection=deepcopy(prior)),4


class Contracts(unittest.TestCase):
    def test_five_frozen_runtime_helpers_unchanged(self):
        frozen=json.loads(RUNTIME_MAP.read_bytes())['helper_sha256']
        self.assertEqual(len(frozen),5)
        self.assertTrue(all(post.sha(p)==h for p,h in frozen.items()))

    def test_nonzero_runner_return_not_relabelled_success(self):
        values=fixture();before=deepcopy(values)
        self.assertEqual(post.returned_selection(*values),values[1]['selection'])
        self.assertEqual(values,before);self.assertEqual(values[1]['runner_returncode'],2)

    def test_zero_runner_return_remains_unverified(self):
        values=fixture();values[0]['runner_returncode']=values[1]['runner_returncode']=0
        self.assertEqual(post.returned_selection(*values),values[1]['selection'])
        self.assertEqual(values[1]['physical_success'],'unverified')

    def test_exact_decimal_text_stamp_accepted(self):
        values=fixture();values[1]['selection']['stamp_ns']=str(values[1]['selection']['stamp_ns'])
        self.assertEqual(post.returned_selection(*values),values[2]['selection'])

    def test_original_sampler_block_preserved_exactly(self):
        oldpath=W/'.runtime/read_gen59_post_attempt_20261022.py'
        self.assertEqual(post.sha(oldpath),'51a588cfc1ea13f5c9fba5e33ed3373503f1dd3b26011cebc5780abcfeee0a69')
        old=oldpath.read_text();new=Path(post.__file__).read_text()
        begin='        lock, samples, values = threading.Lock(), [], {}'
        end='        after = identities(nodes)'
        a=old[old.index(begin):old.index(end)]
        b=new[new.index(begin):new.index(end)].replace("selection = selection_identity(rospy.get_param('/grasp_mode/selection'))", "selection = parse_selection(rospy.get_param('/grasp_mode/selection'))")
        self.assertEqual(a,b)
        for name in ('sha','verify_sources','identities'):
            find=lambda text:next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
            self.assertEqual(ast.dump(find(old)),ast.dump(find(new)))
        for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
            self.assertIn("'"+topic+"', GoalStatusArray, '/bessica_d_hw_interface')",new)
        self.assertIn("graph_after['publishers'].get(topic)==['/bessica_d_hw_interface']",new)

    def test_only_readonly_calls_and_dynamic17_graph(self):
        tree=ast.parse(Path(post.__file__).read_bytes())
        banned={'publish','set_param','setParam','deleteParam','shutdown','ServiceProxy','execve','Popen','run','system'}
        self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in banned for n in ast.walk(tree)))
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        graph=[n for n in ast.walk(main) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='graph']
        self.assertEqual(len(graph),2)
        self.assertEqual([x.arg for x in graph[0].keywords],['gateway'])
        self.assertEqual([x.arg for x in graph[1].keywords],['gateway','auxiliaries'])
        self.assertIn('source_count=1097',Path(post.__file__).read_text())


def reject(name,mutate):
    def test(self):
        values=fixture();mutate(values)
        with self.assertRaises(Exception):post.returned_selection(*values)
    setattr(Contracts,'test_reject_'+name,test)


reject('entry_not_returned',lambda v:v[0].update(original_runner_returned=False))
reject('boundary_not_passed',lambda v:v[0].update(gate_returned_to_original_mode_line=False))
reject('other_generation_entry',lambda v:v[0].update(kind='GEN59_remote_one_recorded_attempt_recorded'))
reject('automatic_retry_claim',lambda v:v[0].update(automatic_retry=True))
reject('returncode_mismatch',lambda v:v[1].update(runner_returncode=3))
reject('returncode_bool',lambda v:(v[0].update(runner_returncode=0),v[1].update(runner_returncode=False)))
reject('different_exact_stamp',lambda v:v[2]['selection'].update(stamp_ns=v[2]['selection']['stamp_ns']+1))
reject('wrong_generation',lambda v:(v[1]['selection'].update(generation=5),v[2]['selection'].update(generation=5)))
reject('old_stamp',lambda v:(v[1]['selection'].update(stamp_ns=v[3]['selection']['stamp_ns']),v[2]['selection'].update(stamp_ns=v[3]['selection']['stamp_ns'])))
reject('prior_changed',lambda v:v[2]['previous_selection'].update(stamp_ns=v[2]['previous_selection']['stamp_ns']+1))
reject('missing_field',lambda v:v[1]['selection'].pop('strategy'))
reject('extra_field',lambda v:v[1]['selection'].update(extra=1))
reject('lossy_float_stamp',lambda v:v[1]['selection'].update(stamp_ns=float(v[1]['selection']['stamp_ns'])))
reject('selection_request_failed',lambda v:v[2]['selection_response'].update(success=False))
reject('commit_unacknowledged',lambda v:v[2]['selection_commit'].update(latched_selection_matched=False))
reject('physical_success_overclaim',lambda v:v[1].update(physical_success='success'))


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    out=args.output.resolve();assert post.E60 in out.parents and Path('/media/zhuyupei/SanStand').is_mount()
    out.mkdir();guard=OfflineGuard(out);sys.addaudithook(guard)
    started=time.monotonic();stream=io.StringIO();suite=unittest.defaultTestLoader.loadTestsFromTestCase(Contracts)
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (out/'tests.log').write_text(stream.getvalue())
    summary=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        skipped=len(result.skipped),elapsed_sec=time.monotonic()-started,helper=str(Path(post.__file__).resolve()),
        helper_sha256=post.sha(post.__file__),test_helper_sha256=post.sha(__file__),blocked_operations=guard.blocked,
        runtime_map_path=str(RUNTIME_MAP),runtime_map_sha256=post.sha(RUNTIME_MAP),
        five_runtime_helpers_unchanged=all(post.sha(p)==h for p,h in json.loads(RUNTIME_MAP.read_bytes())['helper_sha256'].items()),
        ros_calls_performed=False,network_calls_performed=False,
        motion_commands_sent=False,actual_post_attempt_state_observed=False,
        scope='synthetic returned-selection proof, exact inherited 20SDK block, stronger action owner and static readonly effects')
    (out/'result.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print(json.dumps(summary))
    if not result.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
